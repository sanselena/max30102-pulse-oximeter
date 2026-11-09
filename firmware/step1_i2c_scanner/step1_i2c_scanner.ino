// Step 1 (Table 10, step 3): I2C scanner.
// Lists every device that answers on the I2C bus, every 3 seconds.
// Expected with the MAX30102 wired through the level shifter: one device at 0x57.
// Open Tools > Serial Monitor at 115200 baud.

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  // If SDA or SCL is stuck low (short to GND, missing power), give up after 3 ms
  // instead of freezing the Uno.
  Wire.setWireTimeout(3000, true);
#endif
  Serial.println(F("I2C scanner - looking for the MAX30102 at 0x57"));
}

void loop() {
  uint8_t found = 0;
  bool timedOut = false;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print(F("  device at 0x"));
      if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX);
      if (addr == 0x57) Serial.print(F("  <- MAX30102"));
      Serial.println();
      found++;
    } else if (err == 5) {
      timedOut = true;
      break;
    }
  }

  if (timedOut) {
    Serial.println(F("Bus timeout: SDA or SCL is held low. Unplug USB and check the wiring."));
  } else if (found == 0) {
    Serial.println(F("No devices found. Check power, GND, SDA/SCL and the level shifter."));
  } else {
    Serial.print(F("Done, "));
    Serial.print(found);
    Serial.println(F(" device(s) found."));
  }
  Serial.println();
  delay(3000);
}
