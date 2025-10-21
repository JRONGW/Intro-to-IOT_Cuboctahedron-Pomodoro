#include <Wire.h>
#include "MPU6050.h"   // use the one you already have

MPU6050 mpu;
volatile bool isrFlag = false;
const byte INT_PIN = 2; // D2 (~2) on MKR1010

// --- Minimal I2C register read helper (no getByte in this lib)
uint8_t readReg(uint8_t dev, uint8_t reg) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(dev, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

// Rowberg address if AD0=LOW is 0x68. If your board is 0x69, change here:
uint8_t mpuAddr = 0x68;

// MPU-6050 register addresses
#define MPU6050_RA_WHO_AM_I     0x75
#define MPU6050_RA_PWR_MGMT_1   0x6B
#define MPU6050_RA_INT_ENABLE   0x38
#define MPU6050_RA_INT_PIN_CFG  0x37
#define MPU6050_RA_INT_STATUS   0x3A

void isr() { isrFlag = true; }

void setup() {
  Serial.begin(115200); while (!Serial) {}
  Serial.println("\n=== MPU INT debug (no getByte) ===");

  Wire.begin();
  Wire.setClock(400000);

  pinMode(INT_PIN, INPUT_PULLDOWN);   // SAMD supports pulldown

  // Init device
  mpu.initialize();

  // Sanity: if your lib has this, use it; otherwise skip
  if (mpu.testConnection()) {
    Serial.println("MPU connected");
  } else {
    Serial.println("MPU connect FAIL");
    // still continue; we can try to read registers directly
  }

  // Make sure not sleeping
  // Some libs have setSleepEnabled(false); if not, write PWR_MGMT_1 directly:
  // mpu.setSleepEnabled(false);
  // Direct write: clear sleep bit
  // (Only if your lib lacks the setter — uncomment if needed)
  // Wire.beginTransmission(mpuAddr); Wire.write(MPU6050_RA_PWR_MGMT_1); Wire.write(0x00); Wire.endTransmission();

  // --- Configure INT pin behavior via library if available ---
  // Many variants expose these; if yours doesn’t, skip — defaults often still work.
  // Active HIGH:
  #ifdef MPU6050_INCLUDE_DMP_MOTIONAPPS20 // just to hint variants; safe to call anyway
  #endif
  mpu.setInterruptMode(false);          // false = active HIGH (if available)
  mpu.setInterruptDrive(false);         // false = push-pull (if available)
  mpu.setInterruptLatch(true);          // latch until cleared (if available)
  mpu.setInterruptLatchClear(true);     // clear on INT_STATUS read (if available)

  // --- Enable DATA_RDY interrupt ---
  // If your lib has setIntEnabled(mask):
  mpu.setIntEnabled(0x01);              // bit0 = DATA_RDY
  // If it doesn't, direct write fallback (uncomment if needed):
  // Wire.beginTransmission(mpuAddr); Wire.write(MPU6050_RA_INT_ENABLE); Wire.write(0x01); Wire.endTransmission();

  attachInterrupt(digitalPinToInterrupt(INT_PIN), isr, RISING);

  // Dump key registers using direct reads
  uint8_t who = readReg(mpuAddr, MPU6050_RA_WHO_AM_I);
  uint8_t pwr = readReg(mpuAddr, MPU6050_RA_PWR_MGMT_1);
  uint8_t ien = readReg(mpuAddr, MPU6050_RA_INT_ENABLE);
  uint8_t ipc = readReg(mpuAddr, MPU6050_RA_INT_PIN_CFG);

  Serial.print("WHO_AM_I    = 0x"); Serial.println(who, HEX);
  Serial.print("PWR_MGMT_1  = 0x"); Serial.println(pwr, HEX);
  Serial.print("INT_ENABLE  = 0x"); Serial.println(ien, HEX);
  Serial.print("INT_PIN_CFG = 0x"); Serial.println(ipc, HEX);

  Serial.println("alive");
}

void loop() {
  // POLL path: check INT_STATUS whether or not the wire works
  uint8_t st = readReg(mpuAddr, MPU6050_RA_INT_STATUS);  // also clears latched INT if configured so
  if (st) {
    Serial.print("INT_STATUS=0x"); Serial.println(st, HEX);
    if (st & 0x01) {
      // Read one sample via library to prove data is flowing
      int16_t ax, ay, az, gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      // (optional) Serial.println(ax);
    }
  }

  // ISR path: did D2 see a rising edge?
  if (isrFlag) {
    isrFlag = false;
    Serial.println("ISR! (D2 saw a rising edge)");
  }

  delay(10);
}
