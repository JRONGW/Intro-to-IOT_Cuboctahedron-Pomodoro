// That’s an I²C bus scanner. It tests whether anything is alive on the I²C lines and what 7-bit addresses they respond to.
// What it verifies:
//   Wiring & power: SDA/SCL connected correctly, device powered, grounds tied together.
//   Device presence & address: prints any responding address (e.g., MPU-6050 at 0x68; 0x69 if AD0=HIGH).
//   Bus health: if nothing shows, there may be shorts, missing pull-ups, wrong voltage, or a stuck device.


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
