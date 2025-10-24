/***************************************************************
 * Cuboctahedron Face Detection via Gravity Vector (MPU-6050 DMP)
 * Board: Arduino MKR WiFi 1010 (works on others too)
 * Sensor: GY-521 / MPU-6050 (Jeff Rowberg DMP)
 *
 * Prints ONLY when the detected face CHANGES and then remains
 * the same for 10 seconds (dwell-based confirmation).
 ***************************************************************/

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "helper_3dmath.h"   // Quaternion, VectorFloat

/********* SENSOR *********/
MPU6050 mpu;                 // if AD0=HIGH use: MPU6050 mpu(0x69);
bool dmpReady = false;
uint16_t packetSize = 0;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat grav;

/********* FACE NORMALS *********/
// 6 square face outward normals
const float N6[6][3] = {
  { 1, 0, 0}, {-1, 0, 0},
  { 0, 1, 0}, { 0,-1, 0},
  { 0, 0, 1}, { 0, 0,-1}
};
const char* N6name[6] = {"+X","-X","+Y","-Y","+Z","-Z"};

// 8 triangle face outward normals (unit length)
const float s = 0.57735026919f; // 1/sqrt(3)
const float N8[8][3] = {
  { s, s, s}, { s, s,-s}, { s,-s, s}, { s,-s,-s},
  {-s, s, s}, {-s, s,-s}, {-s,-s, s}, {-s,-s,-s}
};
const char* N8name[8] = {
  "+ + +","+ + -","+ - +","+ - -",
  "- + +","- + -","- - +","- - -"
};

// Hysteresis to prevent flicker
const float ENTER_TH = 0.85f;   // enter a face only if -dot(n,g) >= 0.85
const float EXIT_TH  = 0.80f;   // leave current face if score drops below this

// Optional simple low-pass filter for gravity vector (0..1)
const float LPF_ALPHA = 0.2f;   // 0=no filter; 0.2 is mild smoothing

// Dwell logic (announce only after stable for this many ms)
const uint32_t DWELL_MS = 10000UL;  // 10 seconds

struct FaceState {
  bool  isSquare;
  int   idx;
  float score;
};

FaceState currentFace = { true, -1, -1.0f };

// For low-pass gravity
bool haveLPF = false;
float gx_f = 0, gy_f = 0, gz_f = 0;

// Dwell tracking
FaceState dwellCandidate = { true, -1, -1.0f };
uint32_t  dwellStartMs   = 0;
bool      announcedThisFace = false;

/********* SENSOR FRAME -> BODY FRAME CALIBRATION *********
 * If the sensor isn't perfectly aligned with your object, edit
 * this function to rotate (x,y,z) into your body frame.
 * Default: identity (no rotation).
 **********************************************************/
void applyBodyRotation(float &x, float &y, float &z) {
  // Identity mapping; edit if needed.
}

/********* (REMOVED/COMMENTED) TIMER FUNCTION *********/
// void startPomodoroTimer() { /* removed for now */ }
// void stopPomodoroTimer()  { /* removed for now */ }

/********* UTIL *********/
static inline float invSqrt(float x) { return 1.0f / sqrtf(x); }

static inline bool sameFace(const FaceState& a, const FaceState& b) {
  return (a.idx == b.idx) && (a.isSquare == b.isSquare);
}

/********* SETUP *********/
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  // Wire.setClock(400000);  // Uncomment for fast I2C if stable

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 not connected! Check wiring/power."));
  }

  int devStatus = mpu.dmpInitialize();
  // Optional: set your own offsets here if you have them
  // mpu.setXAccelOffset(...); mpu.setYAccelOffset(...); mpu.setZAccelOffset(...);
  // mpu.setXGyroOffset(...);  mpu.setYGyroOffset(...);  mpu.setZGyroOffset(...);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println(F("DMP ready."));
  } else {
    Serial.print(F("DMP init failed: ")); Serial.println(devStatus);
  }

  Serial.println(F("Cuboctahedron face detection with 10s dwell confirmation."));
}

/********* MAIN LOOP *********/
void loop() {
  if (!dmpReady) return;

  // Handle FIFO overflow gracefully
  uint16_t fifoCount = mpu.getFIFOCount();
  if (fifoCount == 1024) {
    mpu.resetFIFO();
    return;
  }
  if (fifoCount < packetSize) return;

  // Read exactly one DMP packet
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // Quaternion -> Gravity (body frame)
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&grav, &q);

  // Normalize gravity
  float gx = grav.x, gy = grav.y, gz = grav.z;
  float gnorm = sqrtf(gx*gx + gy*gy + gz*gz);
  if (gnorm > 0.0f) {
    float invg = 1.0f / gnorm;
    gx *= invg; gy *= invg; gz *= invg;
  }

  // Apply fixed body rotation (if your sensor is mounted skewed)
  applyBodyRotation(gx, gy, gz);

  // Optional low-pass filtering for stability
  if (!haveLPF) {
    gx_f = gx; gy_f = gy; gz_f = gz; haveLPF = true;
  } else {
    gx_f = (1.0f - LPF_ALPHA) * gx_f + LPF_ALPHA * gx;
    gy_f = (1.0f - LPF_ALPHA) * gy_f + LPF_ALPHA * gy;
    gz_f = (1.0f - LPF_ALPHA) * gz_f + LPF_ALPHA * gz;
    // Renormalize after LPF
    float n2 = gx_f*gx_f + gy_f*gy_f + gz_f*gz_f;
    float invn = invSqrt(n2);
    gx_f *= invn; gy_f *= invn; gz_f *= invn;
  }

  // Find best face by maximizing -dot(n, g)
  float bestScore = -1e9f;
  int   bestIdx   = -1;
  bool  bestIsSquare = true;

  // Squares (6)
  for (int i = 0; i < 6; i++) {
    float dotv = N6[i][0]*gx_f + N6[i][1]*gy_f + N6[i][2]*gz_f;
    float score = -dotv;
    if (score > bestScore) { bestScore = score; bestIdx = i; bestIsSquare = true; }
  }
  // Triangles (8)
  for (int i = 0; i < 8; i++) {
    float dotv = N8[i][0]*gx_f + N8[i][1]*gy_f + N8[i][2]*gz_f;
    float score = -dotv;
    if (score > bestScore) { bestScore = score; bestIdx = i; bestIsSquare = false; }
  }

  // Hysteresis/State machine for candidate face
  if (currentFace.idx == -1) {
    if (bestScore > ENTER_TH) {
      currentFace.isSquare = bestIsSquare;
      currentFace.idx      = bestIdx;
      currentFace.score    = bestScore;
    }
  } else {
    if (bestIsSquare == currentFace.isSquare && bestIdx == currentFace.idx) {
      currentFace.score = bestScore;
    } else {
      if (bestScore > ENTER_TH || currentFace.score < EXIT_TH) {
        currentFace.isSquare = bestIsSquare;
        currentFace.idx      = bestIdx;
        currentFace.score    = bestScore;
      }
    }
  }

  // Dwell logic: start or continue timing only when a candidate exists and strong enough
  bool haveCandidate = (currentFace.idx != -1) && (currentFace.score > ENTER_TH);

  if (!haveCandidate) {
    // No valid face yet—reset dwell tracking
    dwellCandidate.idx = -1;
    announcedThisFace  = false;
    return;
  }

  uint32_t nowMs = millis();

  if (dwellCandidate.idx == -1 || !sameFace(dwellCandidate, currentFace)) {
    // New candidate face => start dwell timing
    dwellCandidate = currentFace;
    dwellStartMs   = nowMs;
    announcedThisFace = false;
  } else {
    // Same candidate; check dwell time
    uint32_t elapsed = nowMs - dwellStartMs;
    if (!announcedThisFace && elapsed >= DWELL_MS) {
      // Announce once after 10 seconds of stability
      if (dwellCandidate.isSquare) {
        Serial.print(F("[CONFIRMED 10s] Square face DOWN: "));
        Serial.print(N6name[dwellCandidate.idx]);
        Serial.print(F("   score=")); Serial.println(dwellCandidate.score, 3);
      } else {
        Serial.print(F("[CONFIRMED 10s] Triangle face DOWN (signs XYZ): "));
        Serial.print(N8name[dwellCandidate.idx]);
        Serial.print(F("   score=")); Serial.println(dwellCandidate.score, 3);
      }
      announcedThisFace = true;
    }
  }
}
