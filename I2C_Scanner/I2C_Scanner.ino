#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("I2C Scanner (MKR WiFi 1010)");
  Wire.begin();
}

void loop() {
  byte count = 0;
  Serial.println("\nScanning...");
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Found I2C device at 0x");
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      count++;
    } else if (error == 4) {
      Serial.print("Unknown error at 0x");
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
    }
  }
  if (count == 0) Serial.println("No I2C devices found.");
  else            Serial.println("Scan done.");
  delay(2000);
}
