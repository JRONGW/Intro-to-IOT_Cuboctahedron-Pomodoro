/***************************************************************
 * MKR WiFi 1010 + MPU-6050 (GY-521) — Cuboctahedron Face Detection
 * + WiFiNINA + MQTT (tcp 1884) + 72-pixel Website Effects
 *
 * Detection (accelerometer-only):
 *  - Hold a face steady for 10 s (DWELL_MS) to confirm.
 *  - Re-arms immediately after confirmation.
 *  - Suppresses re-trigger on the SAME face; requires a DIFFERENT face next.
 *
 * Effects on website grid (72 pixels, publishes 216-byte RGB frames):
 *  - Square DOWN   → PINK↔WHITE ROW scan for 20 s → YELLOW chaser (0→71 loop)
 *  - Triangle DOWN → BLUE↔WHITE COLUMN scan for 10 s → YELLOW chaser
 *
 * MQTT:
 *   host: mqtt.cetools.org
 *   port: 1884 (TCP)
 *   publish topic: student/CASA0014/luminaire/<USER>
 *   listens for: student/CASA0014/luminaire/user, .../brightness
 ***************************************************************/
#include <Wire.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#include <SPI.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <utility/wifi_drv.h>

#include "arduino_secrets.h"  // SECRET_SSID / SECRET_PASS (+ optional MQTT_USERNAME / MQTT_PASSWORD)

/********* WIFI *********/
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

/********* MQTT *********/
static const char* MQTT_HOST     = "mqtt.cetools.org";
static const int   MQTT_PORT     = 1884;  // use 1884
static const char* MQTT_CLIENTID = "MKR1010_Cubo_FaceEffects";

// Optional auth via arduino_secrets.h:
//   #define MQTT_USERNAME "user"
//   #define MQTT_PASSWORD "pass"
#ifdef MQTT_USERNAME
  #define HAS_MQTT_AUTH 1
#else
  #define HAS_MQTT_AUTH 0
#endif

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

/********* Topics *********/
const char* mqtt_cmd_topic          = "student/CASA0014/luminaire/cmd";
const char* mqtt_base_topic         = "student/CASA0014/luminaire";
const char* user_update_topic       = "student/CASA0014/luminaire/user";
const char* brightness_update_topic = "student/CASA0014/luminaire/brightness";

int  LUMINAIRE_USER       = 25;   // change if needed
int  LUMINAIRE_BRIGHTNESS = 150;  // 0..255 (used for local mirror only)
char mqtt_data_topic[64];         // student/CASA0014/luminaire/<user>

/********* Website “virtual strip” *********/
#define NEOPIXEL_COUNT 72
#define NEOPIXEL_DATA_LENGTH (NEOPIXEL_COUNT * 3)
uint8_t frameBuf[NEOPIXEL_DATA_LENGTH];

/********* Optional local mirror strip *********/
#define NEOPIXEL_PIN 6
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

/********* IMU *********/
MPU6050 mpu;   // default address 0x68

/********* FACE NORMALS (like your IMU-only sketch) *********/
// 6 square-face outward normals
const float N6[6][3] = {
  { 1, 0, 0}, {-1, 0, 0},
  { 0, 1, 0}, { 0,-1, 0},
  { 0, 0, 1}, { 0, 0,-1}
};
const char* N6name[6] = {"+X","-X","+Y","-Y","+Z","-Z"};

// 8 triangle-face outward normals (unit)
const float sN = 0.57735026919f; // 1/sqrt(3)
const float N8[8][3] = {
  { sN, sN, sN}, { sN, sN,-sN}, { sN,-sN, sN}, { sN,-sN,-sN},
  {-sN, sN, sN}, {-sN, sN,-sN}, {-sN,-sN, sN}, {-sN,-sN,-sN}
};
const char* N8name[8] = {
  "+ + +","+ + -","+ - +","+ - -",
  "- + +","- + -","- - +","- - -"
};

/********* Detection thresholds & timing *********/
const float ENTER_TH     = 0.80f;
const float EXIT_TH      = 0.72f;        // kept for stability
const float LPF_ALPHA    = 0.20f;        // accel LPF (0..1)
const unsigned long DWELL_MS      = 10000UL; // 10 s confirm
const unsigned long REARM_DROP_MS = 800UL;   // unused for hard re-arm, kept for robustness

struct FaceState { bool isSquare; int idx; float score; };

static inline float invSqrt(float x) { return 1.0f / sqrtf(x); }
static inline bool sameFace(const FaceState& a, const FaceState& b) {
  return (a.idx == b.idx) && (a.isSquare == b.isSquare);
}

/*** Detection state ***/
FaceState currentFace      = { true, -1, -1.0f };
bool      haveLPF          = false;
float     gx_f=0, gy_f=0, gz_f=0;

FaceState dwellCandidate   = { true, -1, -1.0f };
unsigned long dwellStartMs = 0;

// last confirmed (used to suppress same-face retrigger)
FaceState lastConfirmed    = { true, -1, -1.0f };

// optional re-arm on dips (robustness)
unsigned long belowStartMs = 0;
bool belowActive           = false;

/********* Effect program *********/
const unsigned long DURATION_SQUARE_MS   = 20000UL;
const unsigned long DURATION_TRIANGLE_MS = 10000UL;

const unsigned long SCAN_PERIOD_MS  = 150UL;  // step rate for scans
const unsigned long CHASE_PERIOD_MS = 80UL;   // yellow chaser step rate

bool programRunning=false, programIsSquare=false;
unsigned long programStart=0, effectStartMs=0;

enum EffectMode { EFFECT_IDLE, EFFECT_SQUARE, EFFECT_TRIANGLE, EFFECT_YELLOW };
EffectMode effect = EFFECT_IDLE;

// Scan & chaser indices
int scanRow = 0;          // 0..5 (top→bottom) for square
int scanCol = 0;          // 0..11 (left→right) for triangle  <<< NEW
int chaseIdx = 0;         // 0..71
unsigned long lastStepMs = 0;

/********* Onboard RGB helpers (via NINA) — 25=R, 26=G, 27=B *********/
static inline void setRGB(uint8_t r,uint8_t g,uint8_t b){
  WiFiDrv::pinMode(25, OUTPUT);
  WiFiDrv::pinMode(26, OUTPUT);
  WiFiDrv::pinMode(27, OUTPUT);
  WiFiDrv::analogWrite(25, r);
  WiFiDrv::analogWrite(26, g);
  WiFiDrv::analogWrite(27, b);
}
static inline void LedRed()    { setRGB(155,0,0); }
static inline void LedGreen()  { setRGB(0,155,0); }
static inline void LedBlue()   { setRGB(0,0,155); }
static inline void LedYellow() { setRGB(255,255,0); }
static inline void LedWhite()  { setRGB(255,255,255); }
static inline void LedPink()   { setRGB(255,64,128); }

/********* Website frame helpers *********/
void fillSolidFrame(uint8_t r, uint8_t g, uint8_t b){
  for (int i=0;i<NEOPIXEL_COUNT;i++){ frameBuf[i*3+0]=r; frameBuf[i*3+1]=g; frameBuf[i*3+2]=b; }
}
static inline int idxFromRowCol(int r, int c){ return c*6 + r; } // 6 rows × 12 cols

void renderRowScanFrame(int r, uint8_t rr,uint8_t rg,uint8_t rb, uint8_t br,uint8_t bg,uint8_t bb){
  fillSolidFrame(br,bg,bb);
  for (int c=0;c<12;c++){
    int i = idxFromRowCol(r,c);
    frameBuf[i*3+0]=rr; frameBuf[i*3+1]=rg; frameBuf[i*3+2]=rb;
  }
}

// NEW: column scan helper
void renderColScanFrame(int c, uint8_t cr,uint8_t cg,uint8_t cb, uint8_t br,uint8_t bg,uint8_t bb){
  fillSolidFrame(br,bg,bb);
  if (c<0) c=0; if (c>11) c=11;
  for (int r=0;r<6;r++){
    int i = idxFromRowCol(r,c);
    frameBuf[i*3+0]=cr; frameBuf[i*3+1]=cg; frameBuf[i*3+2]=cb;
  }
}

void renderChaserFrame(int i, uint8_t r,uint8_t g,uint8_t b){
  fillSolidFrame(0,0,0);
  if (i<0) i=0; if (i>=NEOPIXEL_COUNT) i=NEOPIXEL_COUNT-1;
  frameBuf[i*3+0]=r; frameBuf[i*3+1]=g; frameBuf[i*3+2]=b;
}

bool publishFrame(){
  if (!mqttClient.connected()){
    Serial.println("[MQTT] Not connected; frame NOT published.");
    return false;
  }
  bool ok = mqttClient.publish(mqtt_data_topic, frameBuf, NEOPIXEL_DATA_LENGTH);
  Serial.print("[PUB] "); Serial.print(NEOPIXEL_DATA_LENGTH); Serial.print("B -> "); Serial.println(mqtt_data_topic);
  if (!ok){ Serial.print("  state="); Serial.println(mqttClient.state()); }
  return ok;
}
void showLocalMirror(){
  pixels.setBrightness(LUMINAIRE_BRIGHTNESS);
  for (int i=0;i<NEOPIXEL_COUNT;i++){ pixels.setPixelColor(i, frameBuf[i*3+0], frameBuf[i*3+1], frameBuf[i*3+2]); }
  pixels.show();
}
void publishSolidAndShow(uint8_t r,uint8_t g,uint8_t b){
  fillSolidFrame(r,g,b); publishFrame(); setRGB(r,g,b); showLocalMirror();
}

/********* MQTT status JSON (optional telemetry) *********/
void publishCmdJSON(const char* palette, uint16_t seconds, const char* faceName){
  char buf[200];
  snprintf(buf, sizeof(buf),
    "{\"device\":\"%s\",\"cmd\":\"timer\",\"palette\":\"%s\",\"seconds\":%u,\"face\":\"%s\"}",
    MQTT_CLIENTID, palette, seconds, faceName ? faceName : "");
  mqttClient.publish(mqtt_cmd_topic, buf);
}
void publishState(const char* faceName, const char* faceType, float score, bool valid, bool locked){
  char buf[220];
  snprintf(buf, sizeof(buf),
    "{\"device\":\"%s\",\"cmd\":\"state\",\"face\":\"%s\",\"type\":\"%s\",\"score\":%.3f,\"valid\":%s,\"locked\":%s}",
    MQTT_CLIENTID, faceName ? faceName : "none", faceType ? faceType : "none",
    score, valid ? "true":"false", locked ? "true":"false");
  mqttClient.publish(mqtt_cmd_topic, buf);
}

/********* Wi-Fi *********/
void ensureWiFi(){
  if (WiFi.status() == WL_CONNECTED) return;
  if (WiFi.status() == WL_NO_MODULE) { Serial.println("WiFi module not present!"); return; }
  LedBlue();
  Serial.print("Connecting WiFi: "); Serial.println(ssid);
  int st = WL_IDLE_STATUS;
  while (st != WL_CONNECTED){ st = WiFi.begin(ssid, pass); delay(500); Serial.print("."); }
  Serial.println(); Serial.print("WiFi OK, IP: "); Serial.println(WiFi.localIP());
  LedGreen();
}

/********* MQTT *********/
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, user_update_topic) == 0){
    char s[16]; unsigned n = min(length, (unsigned)(sizeof(s)-1)); memcpy(s,payload,n); s[n]='\0';
    int new_user = atoi(s);
    if (new_user != LUMINAIRE_USER){
      mqttClient.unsubscribe(mqtt_data_topic);
      LUMINAIRE_USER = new_user;
      snprintf(mqtt_data_topic, sizeof(mqtt_data_topic), "%s/%d", mqtt_base_topic, LUMINAIRE_USER);
      mqttClient.subscribe(mqtt_data_topic);
      Serial.print("[TOPIC] -> "); Serial.println(mqtt_data_topic);
      publishSolidAndShow(0,0,0); // clear on change
    }
    return;
  }
  if (strcmp(topic, brightness_update_topic) == 0){
    char s[16]; unsigned n = min(length, (unsigned)(sizeof(s)-1)); memcpy(s,payload,n); s[n]='\0';
    int nb = atoi(s); if (nb<0) nb=0; if (nb>255) nb=255;
    LUMINAIRE_BRIGHTNESS = nb; showLocalMirror();
    Serial.print("BRIGHTNESS -> "); Serial.println(LUMINAIRE_BRIGHTNESS);
    return;
  }
}
unsigned long lastConnectionAttempt=0;
const unsigned long RECONNECT_INTERVAL_MS=5000;
void ensureMQTT(){
  if (mqttClient.connected()) return;
  if (millis() - lastConnectionAttempt < RECONNECT_INTERVAL_MS) return;
  lastConnectionAttempt = millis();

  LedBlue();
  Serial.print("MQTT connect "); Serial.print(MQTT_HOST); Serial.print(":"); Serial.println(MQTT_PORT);
  bool ok=false;
#if HAS_MQTT_AUTH
  ok = mqttClient.connect(MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD);
#else
  ok = mqttClient.connect(MQTT_CLIENTID);
#endif
  if (ok){
    mqttClient.subscribe(user_update_topic);
    mqttClient.subscribe(brightness_update_topic);
    snprintf(mqtt_data_topic, sizeof(mqtt_data_topic), "%s/%d", mqtt_base_topic, LUMINAIRE_USER);
    mqttClient.subscribe(mqtt_data_topic);
    Serial.print("[TOPIC] Website/frames: "); Serial.println(mqtt_data_topic);
    LedGreen();
    publishSolidAndShow(0,0,0);
  } else {
    Serial.print("MQTT fail rc="); Serial.println(mqttClient.state());
    LedRed();
  }
}

/********* EFFECT ENGINE *********/
unsigned long programDurationMs(){ return programIsSquare ? DURATION_SQUARE_MS : DURATION_TRIANGLE_MS; }

void updateEffects(){
  unsigned long now=millis(), dur=programDurationMs();
  switch (effect){
    case EFFECT_IDLE: break;

    case EFFECT_SQUARE: { // PINK row-scan on WHITE for 20s
      if (now - effectStartMs >= dur){
        effect = EFFECT_YELLOW; chaseIdx=0; lastStepMs=0;
        publishCmdJSON("yellow_chaser", 0, "square");
        break;
      }
      if (now - lastStepMs >= SCAN_PERIOD_MS){
        lastStepMs = now;
        renderRowScanFrame(scanRow, 255,64,128, 255,255,255);
        publishFrame(); setRGB(255,64,128); showLocalMirror();
        scanRow = (scanRow + 1) % 6;
      }
    } break;

    case EFFECT_TRIANGLE: { // BLUE column-scan on WHITE for 10s  <<< CHANGED
      if (now - effectStartMs >= dur){
        effect = EFFECT_YELLOW; chaseIdx=0; lastStepMs=0;
        publishCmdJSON("yellow_chaser", 0, "triangle");
        break;
      }
      if (now - lastStepMs >= SCAN_PERIOD_MS){
        lastStepMs = now;
        renderColScanFrame(scanCol, 0,0,255, 255,255,255);   // blue column on white
        publishFrame(); setRGB(0,0,155); showLocalMirror();
        scanCol = (scanCol + 1) % 12;                        // advance column 0..11
      }
    } break;

    case EFFECT_YELLOW: { // single-pixel chaser 0..71
      if (now - lastStepMs >= CHASE_PERIOD_MS){
        lastStepMs = now;
        renderChaserFrame(chaseIdx, 255,255,0);
        publishFrame(); setRGB(255,255,0); showLocalMirror();
        chaseIdx = (chaseIdx + 1) % NEOPIXEL_COUNT;
      }
    } break;
  }
}

/********* SETUP *********/
void setup(){
  Serial.begin(115200);
  while(!Serial){}

  LedRed();  // board alive

  // Local mirror
  pixels.begin();
  pixels.setBrightness(LUMINAIRE_BRIGHTNESS);
  pixels.show();

  // MQTT setup
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqtt_callback);
  mqttClient.setBufferSize(512);

  snprintf(mqtt_data_topic, sizeof(mqtt_data_topic), "%s/%d", mqtt_base_topic, LUMINAIRE_USER);
  Serial.print("[TOPIC] Website/frames: "); Serial.println(mqtt_data_topic);

  ensureWiFi();
  ensureMQTT();

  // IMU
  Wire.begin(); Wire.setClock(400000);
  mpu.initialize();
  if (!mpu.testConnection()){
    Serial.println("MPU6050 connection FAIL");
    while(1){ LedRed(); delay(400); LedBlue(); delay(400); }
  }

  LedGreen();
  Serial.println("Hold a face steady for 10 s to confirm. Same face is ignored next time.");
}

/********* LOOP *********/
void loop(){
  ensureWiFi();
  ensureMQTT();
  mqttClient.loop();

  // ---- EFFECTS ALWAYS RUN ----
  updateEffects();

  // ---- IMU READ ----
  int16_t ax, ay, az; mpu.getAcceleration(&ax,&ay,&az);
  float gx = ax/16384.0f, gy = ay/16384.0f, gz = az/16384.0f;

  float n2 = gx*gx + gy*gy + gz*gz;
  if (n2 < 1e-6f){ delay(10); return; }
  float invn = invSqrt(n2); gx*=invn; gy*=invn; gz*=invn;

  // Low-pass + renormalize
  if (!haveLPF){ gx_f=gx; gy_f=gy; gz_f=gz; haveLPF=true; }
  else {
    gx_f=(1.0f-LPF_ALPHA)*gx_f + LPF_ALPHA*gx;
    gy_f=(1.0f-LPF_ALPHA)*gy_f + LPF_ALPHA*gy;
    gz_f=(1.0f-LPF_ALPHA)*gz_f + LPF_ALPHA*gz;
    float m2=gx_f*gx_f+gy_f*gy_f+gz_f*gz_f;
    float invm=invSqrt(m2); gx_f*=invm; gy_f*=invm; gz_f*=invm;
  }

  // --- Best face = argmax -dot(n, g) ---
  float bestScore = -1e9f; int bestIdx=-1; bool bestIsSquare=true;

  // Squares
  for (int i=0;i<6;i++){
    float dotv = N6[i][0]*gx_f + N6[i][1]*gy_f + N6[i][2]*gz_f;
    float score = -dotv;
    if (score > bestScore){ bestScore=score; bestIdx=i; bestIsSquare=true; }
  }
  // Triangles
  for (int i=0;i<8;i++){
    float dotv = N8[i][0]*gx_f + N8[i][1]*gy_f + N8[i][2]*gz_f;
    float score = -dotv;
    if (score > bestScore){ bestScore=score; bestIdx=i; bestIsSquare=false; }
  }

  // Hysteresis tracking for currentFace
  if (currentFace.idx == -1){
    if (bestScore > ENTER_TH){
      currentFace.isSquare=bestIsSquare; currentFace.idx=bestIdx; currentFace.score=bestScore;
    }
  } else {
    if (bestIsSquare == currentFace.isSquare && bestIdx == currentFace.idx){
      currentFace.score = bestScore;
    } else {
      if (bestScore > ENTER_TH || currentFace.score < EXIT_TH){
        currentFace.isSquare=bestIsSquare; currentFace.idx=bestIdx; currentFace.score=bestScore;
        belowActive = false;
      }
    }
  }

  // Candidate?
  bool haveCandidate = (currentFace.idx != -1) && (currentFace.score > ENTER_TH);

  // Suppress SAME face as last confirmed
  if (haveCandidate && lastConfirmed.idx != -1 && sameFace(currentFace, lastConfirmed)){
    dwellCandidate.idx = -1; // don't dwell on same face
    delay(10);
    return;
  }

  // Optional dip re-arm (kept for robustness)
  if (lastConfirmed.idx != -1){
    if (currentFace.score < EXIT_TH){
      if (!belowActive){ belowActive=true; belowStartMs=millis(); }
      else if (millis()-belowStartMs >= REARM_DROP_MS){ belowActive=false; }
    } else { belowActive=false; }
  }

  if (!haveCandidate){
    dwellCandidate.idx = -1;
    delay(10);
    return;
  }

  unsigned long nowMs = millis();

  // Dwell accumulation on new/different face
  if (dwellCandidate.idx == -1 || !sameFace(dwellCandidate, currentFace)){
    dwellCandidate = currentFace;
    dwellStartMs   = nowMs;
  } else {
    unsigned long elapsed = nowMs - dwellStartMs;
    if (elapsed >= DWELL_MS){
      // ------- CONFIRMED -------
      const char* faceType = currentFace.isSquare ? "square" : "triangle";
      const char* faceName = currentFace.isSquare ? N6name[currentFace.idx] : N8name[currentFace.idx];
      Serial.print("[CONFIRMED 10s] "); Serial.print(faceType); Serial.print(" DOWN: ");
      Serial.print(faceName); Serial.print("   score="); Serial.println(currentFace.score, 3);
      publishState(faceName, faceType, currentFace.score, true, true);

      // Start the program for this face (scan then yellow chaser)
      programIsSquare = currentFace.isSquare;
      programRunning  = true;
      programStart    = nowMs;
      lastStepMs      = 0;
      if (programIsSquare){
        scanRow = 0;
        effect = EFFECT_SQUARE; effectStartMs = nowMs;
        publishCmdJSON("pink_white_rowscan", DURATION_SQUARE_MS/1000, faceName);
      } else {
        scanCol = 0;  // start at first column
        effect = EFFECT_TRIANGLE; effectStartMs = nowMs;
        publishCmdJSON("blue_white_colscan", DURATION_TRIANGLE_MS/1000, faceName); // label updated
      }

      // Remember last confirmed to suppress same-face re-trigger
      lastConfirmed = currentFace;

      // Immediately re-arm: reset dwell so a DIFFERENT face can accumulate
      dwellCandidate.idx = -1;
    }
  }

  delay(10);
}
