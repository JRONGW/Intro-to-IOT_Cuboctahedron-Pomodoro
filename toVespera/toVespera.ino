// === WiFi + MQTT ===
#include <WiFiNINA.h>            // MKR1010 WiFi
#include <PubSubClient.h>        // MQTT (install via Library Manager)

// === IMU (Rowberg DMP) ===
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"

// ======================== USER CONFIG ========================
// Wi-Fi
const char* WIFI_SSID = "YOUR_WIFI";
const char* WIFI_PASS = "YOUR_PASS";

// MQTT broker
const char* MQTT_HOST = "mqtt.cetools.org";  // change if different
const uint16_t MQTT_PORT = 1883;             // PubSubClient is TCP, not WS

// Vespera topics (adjust to your class/lab topic scheme)
const char* VESPERA_CMD_TOPIC   = "UCL/90TCR/vespera/cmd";   // one-shot command JSON
const char* VESPERA_RGB_TOPIC   = "UCL/90TCR/vespera/rgb";   // optional streaming "R,G,B"
const bool  STREAM_LIVE_RGB     = false;                     // true => publish RGB ~1.5 s pulse steps

// Device identity (so your messages can be traced)
const char* DEVICE_ID = "mkr1010-cubo-01";

// ====================== FACE/TIMER SETTINGS =================
const unsigned long STABLE_REQUIRED_MS   = 10000UL;  // 10 s to lock a face
const unsigned long DURATION_SQUARE_MS   = 30UL * 1000UL;  // 30 s
const unsigned long DURATION_TRIANGLE_MS = 10UL * 1000UL;  // 10 s
const unsigned long BLINK_PERIOD_MS      = 1500UL;         // only for local preview math (if needed)

// Tolerances (cosines). ~20° for squares, ~15° for triangles
const float COS_THR_SQUARE   = 0.9396926f; // cos(20°)
const float COS_THR_TRIANGLE = 0.9659258f; // cos(15°)

// ========================== GLOBALS =========================
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

MPU6050 mpu;
bool dmpReady = false;
uint16_t packetSize = 0;
uint16_t fifoCount  = 0;
uint8_t  fifoBuffer[64];

Quaternion  q;
VectorFloat gravity;
float ypr[3];

struct Normal {
  float x, y, z;
  bool isSquare;
  const char* name;
};

// 6 axis-aligned (square faces)
Normal SQUARES[6] = {
  {  1,  0,  0, true,  "+X" },
  { -1,  0,  0, true,  "-X" },
  {  0,  1,  0, true,  "+Y" },
  {  0, -1,  0, true,  "-Y" },
  {  0,  0,  1, true,  "+Z" },
  {  0,  0, -1, true,  "-Z" }
};

// 8 body-diagonals (triangle faces), normalized
Normal TRIS[8] = {
  {  0.57735027f,  0.57735027f,  0.57735027f, false, "+++" },
  {  0.57735027f,  0.57735027f, -0.57735027f, false, "++-" },
  {  0.57735027f, -0.57735027f,  0.57735027f, false, "+-+" },
  {  0.57735027f, -0.57735027f, -0.57735027f, false, "+--" },
  { -0.57735027f,  0.57735027f,  0.57735027f, false, "-++" },
  { -0.57735027f,  0.57735027f, -0.57735027f, false, "-+-" },
  { -0.57735027f, -0.57735027f,  0.57735027f, false, "--+" },
  { -0.57735027f, -0.57735027f, -0.57735027f, false, "---" }
};

Normal ALL[14]; // combined: 0..5 squares, 6..13 triangles

// Face state
int currentFace = -1;
int lastFace    = -1;
unsigned long faceChangeTime = 0;
bool faceLocked = false;

// Program state
bool programRunning = false;
bool programIsSquare = false;
unsigned long programStart = 0;

// ========================= HELPERS ==========================
static inline float dot3(float x1,float y1,float z1,float x2,float y2,float z2){
  return x1*x2 + y1*y2 + z1*z2;
}

void buildNormals() {
  for (int i=0;i<6;i++)  ALL[i]   = SQUARES[i];
  for (int i=0;i<8;i++)  ALL[6+i] = TRIS[i];
}

// ---- MQTT helpers ----
void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void ensureMQTT() {
  if (mqtt.connected()) return;
  while (!mqtt.connected()) {
    String cid = String(DEVICE_ID) + "-" + String(random(0xFFFF), HEX);
    if (mqtt.connect(cid.c_str())) break;
    delay(1000);
  }
}

void publishVesperaCommand(const char* palette, uint16_t seconds, const char* faceName) {
  // Generic JSON the Vespera receiver can parse (adjust keys to your house style)
  // Example:
  // { "device":"mkr1010-cubo-01", "cmd":"timer", "palette":"red_yellow", "seconds":30, "face":"+X" }
  char payload[200];
  snprintf(payload, sizeof(payload),
           "{\"device\":\"%s\",\"cmd\":\"timer\",\"palette\":\"%s\",\"seconds\":%u,\"face\":\"%s\"}",
           DEVICE_ID, palette, seconds, faceName ? faceName : "");
  mqtt.publish(VESPERA_CMD_TOPIC, payload);
}

void publishRGB(uint8_t r,uint8_t g,uint8_t b) {
  // CSV "R,G,B" — change if your Vespera expects JSON
  char buf[32];
  snprintf(buf, sizeof(buf), "%u,%u,%u", r,g,b);
  mqtt.publish(VESPERA_RGB_TOPIC, buf);
}

// ========================== SETUP ===========================
void setup() {
  Serial.begin(115200); while(!Serial){}

  // WiFi + MQTT
  ensureWiFi();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  ensureMQTT();

  // IMU
  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU connect FAIL");
    while(1){}
  }
  int status = mpu.dmpInitialize();
  if (status != 0) {
    Serial.print("DMP init fail: "); Serial.println(status);
    while(1){}
  }
  // mpu.dmpSetFIFORate(50); // optional if your header supports it
  mpu.setDMPEnabled(true);

  // (Optional) INT config; we poll anyway
  mpu.setInterruptMode(false);
  mpu.setInterruptDrive(false);
  mpu.setInterruptLatch(true);
  mpu.setInterruptLatchClear(true);
  mpu.setIntEnabled(0x02);

  packetSize = mpu.dmpGetFIFOPacketSize();
  dmpReady = true;

  buildNormals();
  faceChangeTime = millis();

  Serial.print("WiFi IP: "); Serial.println(WiFi.localIP());
  Serial.print("MQTT to ");  Serial.print(MQTT_HOST); Serial.print(":"); Serial.println(MQTT_PORT);
  Serial.print("DMP ready. packetSize="); Serial.println(packetSize);
}

// =========================== LOOP ==========================
void loop() {
  ensureWiFi();
  ensureMQTT();
  mqtt.loop();

  if (!dmpReady) return;

  fifoCount = mpu.getFIFOCount();

  // Overflow guard
  if (fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO overflow -> reset");
    return;
  }

  if (fifoCount < packetSize) {
    delay(2);
    runProgram(); // continue any running program (for streaming RGB)
    return;
  }

  // Drain complete packets
  while (fifoCount >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // Parse one packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);  // debug prints only

    // Debug YPR (deg)
    static uint32_t lastPrint = 0;
    uint32_t now = millis();
    if (now - lastPrint >= 200) { // 5 Hz
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180.0/M_PI); Serial.print('\t');
      Serial.print(ypr[1] * 180.0/M_PI); Serial.print('\t');
      Serial.println(ypr[2] * 180.0/M_PI);
      lastPrint = now;
    }

    // ---- Face detection from gravity ----
    // Rowberg gravity is "down" (body frame); use "up" = -gravity
    float ux = -gravity.x;
    float uy = -gravity.y;
    float uz = -gravity.z;

    // Find best-matching normal
    int bestIdx = -1;
    float bestDot = -2.0f;
    for (int i=0;i<14;i++) {
      float d = dot3(ux, uy, uz, ALL[i].x, ALL[i].y, ALL[i].z);
      if (d > bestDot) { bestDot = d; bestIdx = i; }
    }

    // Validate with per-type threshold
    bool valid = false;
    if (bestIdx >= 0) {
      if (ALL[bestIdx].isSquare) valid = (bestDot >= COS_THR_SQUARE);
      else                        valid = (bestDot >= COS_THR_TRIANGLE);
    }
    int detectedFace = valid ? bestIdx : -1;

    // Stability + program control
    static uint32_t faceChangeTimeLocal = millis();
    if (detectedFace != lastFace) {
      lastFace = detectedFace;
      faceChangeTimeLocal = now;
      faceLocked = false;

      if (programRunning) {
        programRunning = false; // cancel current program on movement
      }
    } else {
      if (!faceLocked && detectedFace >= 0 && (now - faceChangeTimeLocal >= STABLE_REQUIRED_MS)) {
        faceLocked = true;
        currentFace = detectedFace;

        programIsSquare = ALL[currentFace].isSquare;
        programRunning  = true;
        programStart    = now;

        // === SEND COMMAND TO VESPARA ===
        if (programIsSquare) {
          publishVesperaCommand("red_yellow", DURATION_SQUARE_MS/1000, ALL[currentFace].name);
        } else {
          publishVesperaCommand("blue_green", DURATION_TRIANGLE_MS/1000, ALL[currentFace].name);
        }

        Serial.print("Locked face: ");
        Serial.print(ALL[currentFace].name);
        Serial.print(" type=");
        Serial.println(programIsSquare ? "SQUARE (30s)" : "TRIANGLE (10s)");
      }
    }

    runProgram(); // optionally stream live RGB while timer runs
  }
}

// ============== Optional live RGB stream to Vespera ===========
void runProgram() {
  if (!programRunning) return;

  uint32_t now = millis();
  uint32_t elapsed  = now - programStart;
  uint32_t duration = programIsSquare ? DURATION_SQUARE_MS : DURATION_TRIANGLE_MS;

  if (elapsed >= duration) {
    programRunning = false;
    return;
  }

  if (!STREAM_LIVE_RGB) return;

  // Same color logic we described (long fade + pulse), rendered to a single RGB
  // Squares: red -> yellow (R=255, G:0->255, B=0)
  // Triangles: blue -> green (R=0, G:0->255, B=255->0)
  float t = (float)elapsed / (float)duration; if (t<0) t=0; if (t>1) t=1;
  uint8_t r=0,g=0,b=0;

  if (programIsSquare) {
    r = 255;
    g = (uint8_t)(255.0f * t);
    b = 0;
  } else {
    r = 0;
    g = (uint8_t)(255.0f * t);
    b = (uint8_t)(255.0f * (1.0f - t));
  }

  // Blink envelope (triangle wave 0.25..1.0)
  float phase = (now % BLINK_PERIOD_MS) / (float)BLINK_PERIOD_MS;
  float env   = (phase < 0.5f) ? (phase*2.0f) : (2.0f - phase*2.0f);
  float scale = 0.25f + 0.75f * env;

  // Apply envelope
  r = (uint8_t)(r * scale);
  g = (uint8_t)(g * scale);
  b = (uint8_t)(b * scale);

  // Publish current RGB (CSV)
  publishRGB(r,g,b);
}
