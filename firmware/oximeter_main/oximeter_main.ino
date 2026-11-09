// MAX30102 + Arduino Uno pulse oximeter (Progress Report 3, Sections 2.7.2 and 2.7.3).
//
// Every 40 ms the Uno reads the new samples from the sensor's FIFO, splits each channel into
// DC and AC with two exponential moving averages (about 0.5-4.6 Hz), detects beats on the
// inverted IR signal, and computes heart rate and SpO2 = A - B*R. One line is sent per sample.
//
// Output (default, CSV for MATLAB):  t_ms,red,ir,red_ac,ir_ac,hr,spo2,flags
// Lines starting with '#' are messages, not data.
//
// Send one character from the Serial Monitor or Serial Plotter to change things while it runs:
//   c  CSV output (for logging)         p  plot AC signals       w  plot raw signals
//   R / r  red LED current up / down    I / i  IR LED current up / down  (steps of 1 mA)
//   a  next ADC range                   t  die temperature       s  show settings   ?  help

#include <Wire.h>

// ---------------------------------------------------------------------------------------------
// Settings to tune during the tests (Table 10, steps 6-7)
// ---------------------------------------------------------------------------------------------
uint8_t redCurrent = 0x1F;          // LED1 (red) pulse amplitude, 0x1F = about 6.4 mA (Table 7)
uint8_t irCurrent = 0x1F;           // LED2 (IR) pulse amplitude, 0.2 mA per step, max 0xFF = 51 mA
uint8_t adcRange = 1;               // 0 = 2048 nA, 1 = 4096 nA (Table 7), 2 = 8192 nA, 3 = 16384 nA
const long FINGER_DC_MIN = 30000;   // "no finger" if the IR DC is below this; set from first recordings
const float PI_MIN = 0.001;         // "low perfusion" if AC_IR/DC_IR (peak-to-peak) is below 0.1 %
const float CAL_A = 104.0;          // SpO2 = CAL_A - CAL_B * R, starting values from [9]
const float CAL_B = 17.0;           //   (replace with our own values after calibration, Section 2.6)
const bool SWAP_RED_IR = false;     // set true only if the test in the guide shows red and IR swapped

// ---------------------------------------------------------------------------------------------
// MAX30102 registers [4]
// ---------------------------------------------------------------------------------------------
const uint8_t MAX_ADDR = 0x57;
const uint8_t REG_INTR_STATUS_1 = 0x00;
const uint8_t REG_INTR_ENABLE_1 = 0x02;
const uint8_t REG_INTR_ENABLE_2 = 0x03;
const uint8_t REG_FIFO_WR_PTR = 0x04;
const uint8_t REG_OVF_COUNTER = 0x05;
const uint8_t REG_FIFO_RD_PTR = 0x06;
const uint8_t REG_FIFO_DATA = 0x07;
const uint8_t REG_FIFO_CONFIG = 0x08;
const uint8_t REG_MODE_CONFIG = 0x09;
const uint8_t REG_SPO2_CONFIG = 0x0A;
const uint8_t REG_LED1_PA = 0x0C;
const uint8_t REG_LED2_PA = 0x0D;
const uint8_t REG_TEMP_INT = 0x1F;
const uint8_t REG_TEMP_FRAC = 0x20;
const uint8_t REG_TEMP_CONFIG = 0x21;
const uint8_t REG_PART_ID = 0xFF;

// Quality flags, one bit each (Section 2.7.3). Several can be set at once: 3 = no finger + saturated.
const uint8_t FLAG_NO_FINGER = 1;
const uint8_t FLAG_SATURATED = 2;
const uint8_t FLAG_LOW_PERFUSION = 4;
const uint8_t FLAG_AMBIENT = 8;

const long ADC_MAX = 262143;              // 18-bit full scale
const unsigned long POLL_MS = 40;         // FIFO read period
const uint8_t MAX_PER_READ = 5;           // 5 samples x 6 bytes = 30 bytes, fits the 32-byte Wire buffer
const unsigned int SAMPLE_MS = 10;        // 100 samples/s after averaging
const unsigned int REFRACTORY_MS = 300;   // at most 200 beats/min
const unsigned int MAX_INTERVAL_MS = 2000;  // at least 30 beats/min
const unsigned int SETTLE_SAMPLES = 150;  // ignore beats for 1.5 s after the finger is placed

enum OutputMode { OUT_CSV, OUT_PLOT_AC, OUT_PLOT_RAW };
OutputMode outMode = OUT_CSV;

// ---------------------------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------------------------
unsigned long lastPoll = 0;
unsigned long sampleCount = 0;     // samples since start (lost samples included), so t = count * 10 ms
unsigned long lostSamples = 0;
unsigned long i2cErrors = 0;
unsigned long lastErrorPrint = 0;

// Filters: DC is kept multiplied by 32 and AC by 4, so the shifts keep the fractional part.
long redDcS, irDcS, redAcS, irAcS;
bool filtersReady = false;
bool fingerPresent = false;
unsigned int settleLeft = SETTLE_SAMPLES;

// Beat detection
long prevRise = 0;
long peakSinceBeat = 0;     // highest and lowest inverted IR since the last beat
long troughSinceBeat = 0;
long amps[4];
uint8_t ampCount = 0, ampIdx = 0;
long avgAmp = 0;
bool haveLastBeat = false;
unsigned long lastBeatT = 0;
unsigned long lastDecayT = 0;

unsigned int intervals[4];
uint8_t intCount = 0, intIdx = 0;
float rVals[4];
uint8_t rCount = 0, rIdx = 0;
int heartRate = 0;
float spo2 = 0;
bool lowPerfusion = false;

// Per-beat statistics for R
long redMax, redMin, irMax, irMin;
float redDcSum, irDcSum;
unsigned int beatSamples = 0;
uint8_t beatFlags = 0;

// ---------------------------------------------------------------------------------------------
// I2C helpers
// ---------------------------------------------------------------------------------------------
bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MAX_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// Returns the register value, or -1 if the sensor did not answer.
int readReg(uint8_t reg) {
  Wire.beginTransmission(MAX_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(MAX_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

void reportI2cError() {
  i2cErrors++;
  if (millis() - lastErrorPrint > 1000) {
    lastErrorPrint = millis();
    Serial.print(F("# I2C error (total "));
    Serial.print(i2cErrors);
    Serial.println(F("): loose wire or sensor lost power?"));
  }
}

// ---------------------------------------------------------------------------------------------
// Sensor setup (Table 7)
// ---------------------------------------------------------------------------------------------
void writeSpo2Config() {
  // ADC range in bits 6:5, 400 samples/s (011) in bits 4:2, 411 us pulse (11) in bits 1:0.
  // With adcRange = 1 this is 0x2F.
  writeReg(REG_SPO2_CONFIG, (adcRange << 5) | 0x0F);
}

void writeLedCurrents() {
  writeReg(REG_LED1_PA, redCurrent);
  writeReg(REG_LED2_PA, irCurrent);
}

bool setupSensor() {
  if (!writeReg(REG_MODE_CONFIG, 0x40)) return false;  // reset
  unsigned long t0 = millis();
  while (millis() - t0 < 100) {
    int m = readReg(REG_MODE_CONFIG);
    if (m >= 0 && !(m & 0x40)) break;                    // reset bit clears itself when done
  }
  readReg(REG_INTR_STATUS_1);                            // clear the "power ready" flag
  writeReg(REG_FIFO_CONFIG, 0x5F);   // average 4, rollover on, almost-full at 17 samples
  writeSpo2Config();
  writeLedCurrents();
  writeReg(REG_INTR_ENABLE_1, 0x20); // ambient light overflow flag
  writeReg(REG_INTR_ENABLE_2, 0x00);
  writeReg(REG_FIFO_WR_PTR, 0x00);
  writeReg(REG_OVF_COUNTER, 0x00);
  writeReg(REG_FIFO_RD_PTR, 0x00);
  return writeReg(REG_MODE_CONFIG, 0x03);  // SpO2 mode: red and IR
}

// ---------------------------------------------------------------------------------------------
// Signal processing
// ---------------------------------------------------------------------------------------------
void resetDetector() {
  prevRise = 0;
  peakSinceBeat = 0;
  troughSinceBeat = 0;
  ampCount = ampIdx = 0;
  avgAmp = 0;
  haveLastBeat = false;
  intCount = intIdx = 0;
  rCount = rIdx = 0;
  heartRate = 0;
  spo2 = 0;
  lowPerfusion = false;
  beatSamples = 0;
  beatFlags = 0;
  settleLeft = SETTLE_SAMPLES;
}

void startBeatStats(long redAc, long irAc, long redDc, long irDc, uint8_t flags) {
  redMax = redMin = redAc;
  irMax = irMin = irAc;
  redDcSum = redDc;
  irDcSum = irDc;
  beatSamples = 1;
  beatFlags = flags;
}

float medianOf4(float *v, uint8_t n) {
  float s[4];
  for (uint8_t i = 0; i < n; i++) s[i] = v[i];
  for (uint8_t i = 1; i < n; i++) {        // insertion sort, at most 4 values
    float k = s[i];
    int8_t j = i - 1;
    while (j >= 0 && s[j] > k) { s[j + 1] = s[j]; j--; }
    s[j + 1] = k;
  }
  return (n % 2) ? s[n / 2] : (s[n / 2 - 1] + s[n / 2]) / 2;
}

void onBeat(unsigned long t, long redAc, long irAc, long redDc, long irDc, uint8_t flags) {
  // Height (peak minus trough) of the beat that just ended, for the adaptive threshold.
  // One jerk of the finger can be 10x a real beat, so a single beat may at most double the average.
  long amp = peakSinceBeat - troughSinceBeat;
  if (ampCount > 0 && amp > 2 * avgAmp) amp = 2 * avgAmp;
  amps[ampIdx] = amp;
  ampIdx = (ampIdx + 1) % 4;
  if (ampCount < 4) ampCount++;
  long sum = 0;
  for (uint8_t i = 0; i < ampCount; i++) sum += amps[i];
  avgAmp = sum / ampCount;

  if (haveLastBeat && beatSamples > 1) {
    unsigned long interval = t - lastBeatT;
    float acRed = redMax - redMin;
    float acIr = irMax - irMin;
    float dcRed = redDcSum / beatSamples;
    float dcIr = irDcSum / beatSamples;
    float pi = (dcIr > 0) ? acIr / dcIr : 0;
    lowPerfusion = pi < PI_MIN;

    bool clean = beatFlags == 0 && !lowPerfusion && interval >= REFRACTORY_MS &&
                 interval <= MAX_INTERVAL_MS;
    if (clean) {
      intervals[intIdx] = interval;
      intIdx = (intIdx + 1) % 4;
      if (intCount < 4) intCount++;
      if (intCount >= 2) {
        unsigned long isum = 0;
        for (uint8_t i = 0; i < intCount; i++) isum += intervals[i];
        heartRate = (int)(60000UL * intCount / isum);  // 60 / average interval
      }
      if (acIr > 0 && dcRed > 0) {
        rVals[rIdx] = (acRed / dcRed) / (acIr / dcIr);
        rIdx = (rIdx + 1) % 4;
        if (rCount < 4) rCount++;
        spo2 = CAL_A - CAL_B * medianOf4(rVals, rCount);
      }
    }
  }

  haveLastBeat = true;
  lastBeatT = t;
  lastDecayT = t;
  peakSinceBeat = troughSinceBeat = -irAc;
  startBeatStats(redAc, irAc, redDc, irDc, flags);
}

void processSample(long red, long ir, uint8_t flags) {
  unsigned long t = sampleCount * SAMPLE_MS;
  sampleCount++;

  if (red >= ADC_MAX || ir >= ADC_MAX) flags |= FLAG_SATURATED;

  if (!filtersReady) {
    redDcS = red << 5;
    irDcS = ir << 5;
    redAcS = irAcS = 0;
    filtersReady = true;
  }

  // DC: first-order low-pass, alpha = 1/32 (about 0.5 Hz)
  redDcS += red - (redDcS >> 5);
  irDcS += ir - (irDcS >> 5);
  long redDc = redDcS >> 5;
  long irDc = irDcS >> 5;

  // Finger placed: start the DC at the current value instead of waiting for the filter to climb.
  bool finger = irDc >= FINGER_DC_MIN;
  if (finger && !fingerPresent) {
    redDcS = red << 5;
    irDcS = ir << 5;
    redDc = red;
    irDc = ir;
    redAcS = irAcS = 0;
    resetDetector();
  }
  if (!finger && fingerPresent) resetDetector();
  fingerPresent = finger;
  if (!finger) flags |= FLAG_NO_FINGER;

  // AC: raw minus DC, smoothed with alpha = 1/4 (about 4.6 Hz)
  redAcS += (red - redDc) - (redAcS >> 2);
  irAcS += (ir - irDc) - (irAcS >> 2);
  long redAc = redAcS >> 2;
  long irAc = irAcS >> 2;

  if (lowPerfusion) flags |= FLAG_LOW_PERFUSION;

  if (finger) {
    // Per-beat statistics
    if (beatSamples == 0) {
      startBeatStats(redAc, irAc, redDc, irDc, flags);
    } else {
      if (redAc > redMax) redMax = redAc;
      if (redAc < redMin) redMin = redAc;
      if (irAc > irMax) irMax = irAc;
      if (irAc < irMin) irMin = irAc;
      redDcSum += redDc;
      irDcSum += irDc;
      beatSamples++;
      beatFlags |= flags & ~FLAG_LOW_PERFUSION;  // low perfusion is judged per beat, in onBeat()
    }

    // Beat detection on the inverted IR AC: less light during systole, so systole becomes a peak.
    // A beat is the moment the signal has risen 60 % of the average beat height above the lowest
    // point since the previous beat. Measuring from that trough keeps slow drift (breathing,
    // pressure changes) from hiding beats.
    long s = -irAc;
    if (s > peakSinceBeat) peakSinceBeat = s;
    if (s < troughSinceBeat) troughSinceBeat = s;

    if (settleLeft > 0) {
      // First 1.5 s after the finger lands: no beats. The last second gives the first beat height.
      settleLeft--;
      if (settleLeft == 100) peakSinceBeat = troughSinceBeat = s;
      if (settleLeft == 0 && peakSinceBeat > troughSinceBeat) {
        amps[0] = avgAmp = peakSinceBeat - troughSinceBeat;
        ampCount = 1;
        ampIdx = 1;
        peakSinceBeat = troughSinceBeat = s;
      }
      if (settleLeft == 0) lastBeatT = lastDecayT = t;  // start the "no beat for 2 s" clock
      prevRise = 0;
    } else {
      long threshold = avgAmp * 3 / 5;  // 60 % of the average height of the last beats
      if (threshold < 1) threshold = 1;
      long rise = s - troughSinceBeat;
      bool crossed = prevRise < threshold && rise >= threshold;
      prevRise = rise;
      if (crossed && (!haveLastBeat || t - lastBeatT >= REFRACTORY_MS)) {
        onBeat(t, redAc, irAc, redDc, irDc, flags);
        prevRise = 0;
      }
    }

    // No beat for 2 s (finger moved, a large artifact raised the threshold): halve the threshold
    // every second until beats are found again, and stop showing old values.
    if (settleLeft == 0 && t - lastBeatT > MAX_INTERVAL_MS && t - lastDecayT >= 1000) {
      lastDecayT = t;
      peakSinceBeat = troughSinceBeat = s;  // forget the artifact's trough too
      prevRise = 0;
      avgAmp /= 2;
      for (uint8_t i = 0; i < ampCount; i++) amps[i] /= 2;
      heartRate = 0;
      spo2 = 0;
      intCount = intIdx = 0;
    }
  }

  sendSample(t, red, ir, redAc, irAc, flags);
}

void sendSample(unsigned long t, long red, long ir, long redAc, long irAc, uint8_t flags) {
  if (outMode == OUT_CSV) {
    Serial.print(t);
    Serial.print(',');
    Serial.print(red);
    Serial.print(',');
    Serial.print(ir);
    Serial.print(',');
    Serial.print(redAc);
    Serial.print(',');
    Serial.print(irAc);
    Serial.print(',');
    Serial.print(heartRate);
    Serial.print(',');
    Serial.print(spo2, 1);
    Serial.print(',');
    Serial.println(flags);
  } else if (outMode == OUT_PLOT_AC) {
    Serial.print(F("red_ac:"));
    Serial.print(redAc);
    Serial.print(F(",ir_ac:"));
    Serial.println(irAc);
  } else {
    Serial.print(F("red:"));
    Serial.print(red);
    Serial.print(F(",ir:"));
    Serial.println(ir);
  }
}

// ---------------------------------------------------------------------------------------------
// FIFO reading (every 40 ms)
// ---------------------------------------------------------------------------------------------
long read18() {
  long v = (long)Wire.read() << 16;
  v |= (long)Wire.read() << 8;
  v |= Wire.read();
  return v & 0x3FFFF;
}

void pollFifo() {
  int status = readReg(REG_INTR_STATUS_1);  // reading also clears the flags
  int wr = readReg(REG_FIFO_WR_PTR);
  int ovf = readReg(REG_OVF_COUNTER);
  int rd = readReg(REG_FIFO_RD_PTR);
  if (status < 0 || wr < 0 || ovf < 0 || rd < 0) {
    reportI2cError();
    return;
  }

  uint8_t flags = (status & 0x20) ? FLAG_AMBIENT : 0;
  uint8_t pending = (wr - rd) & 0x1F;
  if (ovf > 0) {
    // FIFO was full and old samples were overwritten: read all 32 and keep the time axis right.
    pending = 32;
    lostSamples += ovf;
    sampleCount += ovf;
  }

  while (pending > 0) {
    uint8_t n = pending > MAX_PER_READ ? MAX_PER_READ : pending;
    Wire.beginTransmission(MAX_ADDR);
    Wire.write(REG_FIFO_DATA);
    if (Wire.endTransmission(false) != 0) {
      reportI2cError();
      return;
    }
    uint8_t bytes = n * 6;
    if (Wire.requestFrom(MAX_ADDR, bytes) != bytes) {
      while (Wire.available()) Wire.read();
      reportI2cError();
      return;
    }
    for (uint8_t i = 0; i < n; i++) {
      long a = read18();  // LED1 (red) comes first in SpO2 mode
      long b = read18();  // then LED2 (IR)
      if (SWAP_RED_IR) processSample(b, a, flags);
      else processSample(a, b, flags);
    }
    pending -= n;
  }
}

// ---------------------------------------------------------------------------------------------
// Serial commands
// ---------------------------------------------------------------------------------------------
void printCurrent(const __FlashStringHelper *name, uint8_t code) {
  Serial.print(name);
  Serial.print(code * 0.2, 1);
  Serial.print(F(" mA (0x"));
  if (code < 16) Serial.print('0');
  Serial.print(code, HEX);
  Serial.println(')');
}

void printSettings() {
  printCurrent(F("# red LED: "), redCurrent);
  printCurrent(F("# IR LED:  "), irCurrent);
  Serial.print(F("# ADC range: "));
  Serial.print(2048L << adcRange);
  Serial.println(F(" nA"));
  Serial.print(F("# finger: "));
  Serial.print(fingerPresent ? F("yes") : F("no"));
  Serial.print(F(", IR DC: "));
  Serial.print(irDcS >> 5);
  Serial.print(F(", red DC: "));
  Serial.println(redDcS >> 5);
  Serial.print(F("# lost samples: "));
  Serial.print(lostSamples);
  Serial.print(F(", I2C errors: "));
  Serial.println(i2cErrors);
}

void printTemperature() {
  writeReg(REG_TEMP_CONFIG, 0x01);
  unsigned long t0 = millis();
  while ((readReg(REG_TEMP_CONFIG) & 0x01) && millis() - t0 < 100) {}
  int ti = readReg(REG_TEMP_INT);
  int tf = readReg(REG_TEMP_FRAC);
  Serial.print(F("# die temperature: "));
  if (ti < 0 || tf < 0) {
    Serial.println(F("read failed"));
    return;
  }
  Serial.print((int8_t)ti + (tf & 0x0F) * 0.0625, 2);
  Serial.println(F(" C"));
}

void printHelp() {
  Serial.println(F("# c CSV | p plot AC | w plot raw | R/r red LED +/- | I/i IR LED +/- | a ADC range | t temp | s settings"));
}

uint8_t stepCurrent(uint8_t code, int delta) {
  int v = code + delta;
  if (v < 0) v = 0;
  if (v > 0xFF) v = 0xFF;
  return v;
}

void handleCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    bool changed = false;
    switch (c) {
      case 'c': outMode = OUT_CSV; Serial.println(F("t_ms,red,ir,red_ac,ir_ac,hr,spo2,flags")); break;
      case 'p': outMode = OUT_PLOT_AC; break;
      case 'w': outMode = OUT_PLOT_RAW; break;
      case 'R': redCurrent = stepCurrent(redCurrent, 5); changed = true; break;
      case 'r': redCurrent = stepCurrent(redCurrent, -5); changed = true; break;
      case 'I': irCurrent = stepCurrent(irCurrent, 5); changed = true; break;
      case 'i': irCurrent = stepCurrent(irCurrent, -5); changed = true; break;
      case 'a': adcRange = (adcRange + 1) % 4; writeSpo2Config(); changed = true; break;
      case 't': printTemperature(); break;
      case 's': printSettings(); break;
      case '?': printHelp(); break;
      default: break;  // ignore newlines and anything else
    }
    if (changed) {
      writeLedCurrents();
      filtersReady = false;  // the DC level jumps; restart the filters from the new value
      fingerPresent = false;
      printSettings();
    }
  }
}

// ---------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  Serial.println(F("# MAX30102 pulse oximeter"));

  // Check that the right chip answers (Table 10, step 4); retry until it does.
  while (true) {
    int id = readReg(REG_PART_ID);
    if (id == 0x15 && setupSensor()) break;
    Serial.print(F("# sensor not ready: part ID "));
    if (id < 0) Serial.print(F("no answer"));
    else Serial.print(id, HEX);
    Serial.println(F(" (expected 15). Check wiring, retrying in 2 s"));
    delay(2000);
  }

  printSettings();
  printHelp();
  Serial.println(F("t_ms,red,ir,red_ac,ir_ac,hr,spo2,flags"));
  lastPoll = millis();
}

void loop() {
  handleCommands();
  if (millis() - lastPoll >= POLL_MS) {
    lastPoll += POLL_MS;
    pollFifo();
  }
}
