/***************************************************************
 * MKR WiFi 1010 + MPU-6050 (GY-521) — Cuboctahedron Face Detection
 * Method: Use raw accelerometer as gravity; pick face maximizing -dot(n, g)
 * Prints only after the same face is held 10 seconds.
 * Re-arms after score dips below EXIT_TH for REARM_DROP_MS.
 *
 * MKR1010 I2C: SDA/SCL header pins (3.3V logic). Power GY-521 at 3.3V.
 ***************************************************************/

#include <Wire.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050.h"   // accel-only, no DMP

/********* SENSOR *********/
MPU6050 mpu;           // default 0x68; use MPU6050 mpu(0x69); if AD0=HIGH

/********* FACE NORMALS *********/
// 6 square-face outward normals
const float N6[6][3] = {
  { 1, 0, 0}, {-1, 0, 0},
  { 0, 1, 0}, { 0,-1, 0},
  { 0, 0, 1}, { 0, 0,-1}
};
const char* N6name[6] = {"+X","-X","+Y","-Y","+Z","-Z"};

// 8 triangle-face outward normals (unit)
const float s = 0.57735026919f; // 1/sqrt(3)
const float N8[8][3] = {
  { s, s, s}, { s, s,-s}, { s,-s, s}, { s,-s,-s},
  {-s, s, s}, {-s, s,-s}, {-s,-s, s}, {-s,-s,-s}
};
const char* N8name[8] = {
  "+ + +","+ + -","+ - +","+ - -",
  "- + +","- + -","- - +","- - -"
};

// Thresholds & timing
const float ENTER_TH = 0.80f;
const float EXIT_TH  = 0.72f;
const float LPF_ALPHA = 0.2f;             // accel LPF (0..1)
const unsigned long DWELL_MS = 10000UL;   // 10 s confirm
const unsigned long REARM_DROP_MS = 800UL;// below EXIT_TH this long => re-arm

struct FaceState {
  bool  isSquare;
  int   idx;
  float score;
};

static inline float invSqrt(float x) { return 1.0f / sqrtf(x); }
static inline bool sameFace(const FaceState& a, const FaceState& b) {
  return (a.idx == b.idx) && (a.isSquare == b.isSquare);
}

/*** runtime state ***/
FaceState currentFace = { true, -1, -1.0f };

// Low-pass gravity
bool haveLPF = false;
float gx_f = 0, gy_f = 0, gz_f = 0;

// Dwell tracking
FaceState dwellCandidate = { true, -1, -1.0f };
unsigned long dwellStartMs = 0;
bool announcedThisFace = false;

// Re-arm tracking
unsigned long belowStartMs = 0;
bool belowActive = false;

/********* OPTIONAL: rotate sensor frame -> body frame *********
 * If your sensor is mounted skewed, remap/rotate (x,y,z) here.
 * Default is identity (no rotation).
 **************************************************************/
void applyBodyRotation(float &x, float &y, float &z) {
  // Identity mapping. Edit if needed to match your geometry axes.
}

/********* SETUP *********/
void setup() {
  Serial.begin(115200);
  while(!Serial) {}

  Wire.begin();              // MKR1010: SDA/SCL header pins
  Wire.setClock(400000);     // optional: fast I2C

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 not connected. Check wiring (3.3V power, SDA/SCL, GND)."));
    delay(3000);
  }

  // Optional: stabilize readings for static orientation
  // mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  // mpu.setDLPFMode(3);   // ~44 Hz
  // mpu.setRate(9);       // ~100 Hz

  Serial.println(F("Accel-only face detection. Hold a face steady for 10s to announce."));
}

/********* LOOP *********/
void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Convert to g units (±2g = 16384 LSB/g)
  float gx = ax / 16384.0f;
  float gy = ay / 16384.0f;
  float gz = az / 16384.0f;

  // Normalize vector
  float n2 = gx*gx + gy*gy + gz*gz;
  if (n2 < 1e-6f) { delay(5); return; }
  float invn = invSqrt(n2);
  gx *= invn; gy *= invn; gz *= invn;

  // Body-frame alignment if needed
  applyBodyRotation(gx, gy, gz);

  // Low-pass + renormalize
  if (!haveLPF) {
    gx_f = gx; gy_f = gy; gz_f = gz; haveLPF = true;
  } else {
    gx_f = (1.0f - LPF_ALPHA) * gx_f + LPF_ALPHA * gx;
    gy_f = (1.0f - LPF_ALPHA) * gy_f + LPF_ALPHA * gy;
    gz_f = (1.0f - LPF_ALPHA) * gz_f + LPF_ALPHA * gz;
    float m2 = gx_f*gx_f + gy_f*gy_f + gz_f*gz_f;
    float invm = invSqrt(m2);
    gx_f *= invm; gy_f *= invm; gz_f *= invm;
  }

  // Best face = argmax -dot(n, g)
  float bestScore = -1e9f;
  int   bestIdx   = -1;
  bool  bestIsSquare = true;

  // Squares
  for (int i = 0; i < 6; i++) {
    float dotv = N6[i][0]*gx_f + N6[i][1]*gy_f + N6[i][2]*gz_f;
    float score = -dotv;
    if (score > bestScore) { bestScore = score; bestIdx = i; bestIsSquare = true; }
  }
  // Triangles
  for (int i = 0; i < 8; i++) {
    float dotv = N8[i][0]*gx_f + N8[i][1]*gy_f + N8[i][2]*gz_f;
    float score = -dotv;
    if (score > bestScore) { bestScore = score; bestIdx = i; bestIsSquare = false; }
  }

  // Hysteresis tracking for currentFace
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
        // on face change, drop dwell & rearm state so we can detect again
        belowActive = false;
      }
    }
  }

  // Candidate check for dwell
  bool haveCandidate = (currentFace.idx != -1) && (currentFace.score > ENTER_TH);

  // Re-arm logic
  if (announcedThisFace) {
    if (currentFace.score < EXIT_TH) {
      if (!belowActive) { belowActive = true; belowStartMs = millis(); }
      else if (millis() - belowStartMs >= REARM_DROP_MS) {
        announcedThisFace = false;
        dwellCandidate.idx = -1;
        belowActive = false;
      }
    } else {
      belowActive = false;
    }
  }

  if (!haveCandidate) {
    if (!announcedThisFace) dwellCandidate.idx = -1;
    delay(10);
    return;
  }

  unsigned long nowMs = millis();

  if (dwellCandidate.idx == -1 || !sameFace(dwellCandidate, currentFace)) {
    dwellCandidate = currentFace;
    dwellStartMs   = nowMs;
    announcedThisFace = false;
  } else {
    unsigned long elapsed = nowMs - dwellStartMs;
    if (!announcedThisFace && elapsed >= DWELL_MS) {
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

  delay(10);  // ~100 Hz loop
}
