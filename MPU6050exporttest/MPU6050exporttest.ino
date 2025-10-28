// I²C wiring & power
//   Wire.begin(); Wire.setClock(400000); mpu.initialize(); mpu.testConnection()
//   → Confirms the chip responds on I²C (0x68/0x69) at 400 kHz.
// DMP firmware load & configuration
//   mpu.dmpInitialize(); mpu.setDMPEnabled(true); packetSize = mpu.dmpGetFIFOPacketSize();
//   → Ensures the on-chip DMP successfully boots and outputs fixed-size packets (typically 42 bytes).
// FIFO health
//   fifoCount = mpu.getFIFOCount(); ... if (fifoCount == 1024) resetFIFO();
//   → Checks for overflows
// Quaternion → gravity → YPR math
//   dmpGetQuaternion(&q, fifoBuffer);
//   dmpGetGravity(&gravity, &q);
//   dmpGetYawPitchRoll(ypr, &q, &gravity);
//   → Confirms the DMP orientation solution is coherent and  derives yaw, pitch, roll.
// Streaming/throughput check
//   Prints YPR at ~20 Hz (if (now - last >= 50)) to make sure Serial USB isn’t the bottleneck.


#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"  // Rowberg DMP header

MPU6050 mpu;
bool dmpReady = false;
uint16_t packetSize;  // expected DMP packet size (usually 42)
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU connect FAIL");
    while (1) {}
  }

  // DMP init
  int status = mpu.dmpInitialize();
  if (status != 0) {
    Serial.print("DMP init fail: ");
    Serial.println(status);
    while (1) {}
  }

  // (Optional but helps) lower DMP FIFO rate to reduce overflow if available
  // mpu.dmpSetFIFORate(50); // 50 Hz; comment out if your header lacks this

  mpu.setDMPEnabled(true);

  // Configure INT pin behavior (not strictly needed in polling mode)
  mpu.setInterruptMode(false);       // active HIGH
  mpu.setInterruptDrive(false);      // push-pull
  mpu.setInterruptLatch(true);       // latch until cleared
  mpu.setInterruptLatchClear(true);  // clear on read of INT_STATUS

  // Enable only DMP interrupt bit in case you later use ISR
  mpu.setIntEnabled(0x02);

  packetSize = mpu.dmpGetFIFOPacketSize();
  dmpReady = true;

  Serial.print("DMP ready. packetSize=");
  Serial.println(packetSize);
}

void loop() {
  if (!dmpReady) return;

  fifoCount = mpu.getFIFOCount();

  // Handle overflow immediately
  if (fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO overflow -> reset");
    return;
  }

  if (fifoCount < packetSize) {
    // not enough data yet
    delay(2);
    return;
  }

  // Drain all complete packets
  while (fifoCount >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // Parse one packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Print at a modest rate so Serial doesn’t bottleneck
    static uint32_t last = 0;
    uint32_t now = millis();
    if (now - last >= 50) {  // ~20 Hz print
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180.0 / M_PI);
      Serial.print('\t');
      Serial.print(ypr[1] * 180.0 / M_PI);
      Serial.print('\t');
      Serial.println(ypr[2] * 180.0 / M_PI);
      last = now;
    }
  }
}
