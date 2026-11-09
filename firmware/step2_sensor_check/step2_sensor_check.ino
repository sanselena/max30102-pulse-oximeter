// Step 2 (Table 10, step 4): sensor check.
// Reads the part ID, the revision ID and the die temperature, and tests the INT wire on D2.
// Repeats every 2 seconds, so you can wiggle a wire and see if the check breaks.
// Open Tools > Serial Monitor at 115200 baud.
//
// Expected:  Part ID 0x15 (0x11 means the board carries a MAX30100, not a MAX30102)
//            Temperature close to room temperature (about 20-35 C)
//            INT test: HIGH when idle, LOW when data is waiting -> "INT wire OK"

#include <Wire.h>

const uint8_t MAX_ADDR = 0x57;
const uint8_t INT_PIN = 2;

const uint8_t REG_INTR_STATUS_1 = 0x00;
const uint8_t REG_INTR_ENABLE_1 = 0x02;
const uint8_t REG_MODE_CONFIG = 0x09;
const uint8_t REG_SPO2_CONFIG = 0x0A;
const uint8_t REG_LED1_PA = 0x0C;
const uint8_t REG_LED2_PA = 0x0D;
const uint8_t REG_TEMP_INT = 0x1F;
const uint8_t REG_TEMP_FRAC = 0x20;
const uint8_t REG_TEMP_CONFIG = 0x21;
const uint8_t REG_REV_ID = 0xFE;
const uint8_t REG_PART_ID = 0xFF;

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

void printHex(int v) {
  Serial.print(F("0x"));
  if (v < 16) Serial.print('0');
  Serial.print(v, HEX);
}

void setup() {
  Serial.begin(115200);
  pinMode(INT_PIN, INPUT);  // the level shifter's own resistor pulls the line up
  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  Serial.println(F("MAX30102 sensor check"));
}

void loop() {
  Serial.println(F("----"));

  int id = readReg(REG_PART_ID);
  if (id < 0) {
    Serial.println(F("No answer from 0x57. Run step1_i2c_scanner first."));
    delay(2000);
    return;
  }
  Serial.print(F("Part ID: "));
  printHex(id);
  if (id == 0x15) Serial.println(F("  OK (MAX30102)"));
  else if (id == 0x11) Serial.println(F("  WRONG CHIP: this is a MAX30100. Use the spare board or ask the seller."));
  else Serial.println(F("  unexpected value"));

  Serial.print(F("Revision ID: "));
  printHex(readReg(REG_REV_ID));
  Serial.println();

  // Reset the chip so every check starts from the same state.
  writeReg(REG_MODE_CONFIG, 0x40);
  delay(10);
  readReg(REG_INTR_STATUS_1);  // clears the "power ready" flag

  // Die temperature: start one conversion, wait for it to finish (about 30 ms).
  writeReg(REG_TEMP_CONFIG, 0x01);
  unsigned long t0 = millis();
  while ((readReg(REG_TEMP_CONFIG) & 0x01) && millis() - t0 < 100) {}
  int ti = readReg(REG_TEMP_INT);
  int tf = readReg(REG_TEMP_FRAC);
  if (ti >= 0 && tf >= 0) {
    float temp = (int8_t)ti + (tf & 0x0F) * 0.0625;
    Serial.print(F("Die temperature: "));
    Serial.print(temp, 2);
    Serial.println(F(" C"));
  } else {
    Serial.println(F("Temperature read failed."));
  }

  // INT test. Idle: the line must read HIGH.
  bool idleHigh = digitalRead(INT_PIN) == HIGH;
  // Turn on the "new sample ready" interrupt and start sampling, but do not read the data:
  // after 50 ms the sensor must be holding INT LOW.
  writeReg(REG_SPO2_CONFIG, 0x2F);
  writeReg(REG_LED1_PA, 0x1F);
  writeReg(REG_LED2_PA, 0x1F);
  writeReg(REG_INTR_ENABLE_1, 0x40);
  writeReg(REG_MODE_CONFIG, 0x03);
  delay(50);
  bool activeLow = digitalRead(INT_PIN) == LOW;
  writeReg(REG_MODE_CONFIG, 0x40);  // reset again: LEDs off, interrupts off

  Serial.print(F("INT on D2: idle "));
  Serial.print(idleHigh ? F("HIGH") : F("LOW"));
  Serial.print(F(", data waiting "));
  Serial.print(activeLow ? F("LOW") : F("HIGH"));
  if (idleHigh && activeLow) Serial.println(F("  -> INT wire OK"));
  else if (!idleHigh) Serial.println(F("  -> line stuck LOW: check the INT wire and the shifter channel"));
  else Serial.println(F("  -> line never goes LOW: INT wire not connected to D2 (only matters for the interrupt method)"));

  delay(2000);
}
