#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
  while (!Serial);  // Wait for Serial to open (especially on Teensy)
  Serial.println("I2C Scanner Starting...");

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at 0x");
      Serial.println(address, HEX);
    }
  }

  Serial.println("Scan complete.");
}

void loop() {
  // Nothing in loop
}
