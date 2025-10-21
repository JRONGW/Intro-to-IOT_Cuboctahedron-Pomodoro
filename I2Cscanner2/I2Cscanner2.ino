#include <Wire.h>
void setup() {
  Serial.begin(115200);
  while(!Serial){}
  Serial.println("Scanning I2C...");
  Wire.begin();
}
void loop() {
  byte count=0;
  for (byte addr=1; addr<127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission()==0) {
      Serial.print("Found 0x"); if (addr<16) Serial.print('0');
      Serial.println(addr, HEX); count++;
    }
  }
  if (!count) Serial.println("No I2C devices found.");
  delay(2000);
}
