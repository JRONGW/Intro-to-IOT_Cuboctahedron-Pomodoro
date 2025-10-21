#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin(); // MKR1010 SDA/SCL pins
  // If you set AD0 HIGH (address 0x69), pass it here:
  // if (!mpu.begin(0x69)) { ... }
  if (!mpu.begin(8192)) {
    Serial.println("MPU6050 not found at 8192.");
    while (1) delay(10);
  }
  Serial.println("MPU6050 ready.");

  // Optional: configure ranges & filter
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  Serial.print("Accel (m/s^2): ");
  Serial.print(a.acceleration.x); Serial.print(", ");
  Serial.print(a.acceleration.y); Serial.print(", ");
  Serial.print(a.acceleration.z);

  Serial.print(" | Gyro (rad/s): ");
  Serial.print(g.gyro.x); Serial.print(", ");
  Serial.print(g.gyro.y); Serial.print(", ");
  Serial.print(g.gyro.z);

  Serial.print(" | Temp (°C): ");
  Serial.println(t.temperature);

  delay(200);
}
