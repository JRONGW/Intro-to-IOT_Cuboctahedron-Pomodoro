/***************************************************************
 * MKR WiFi 1010 + MPU-6050 (GY-521) — Cuboctahedron Face Detection
 * + WiFiNINA + MQTT (tcp 1884) + 72-pixel Website Effects
 *
 * Square DOWN   → Row blink-through #F6D5F7 ↔ #FBE9D7 (white→#FBE9D7 bg)
 * Triangle DOWN → Column blink-through #CAD0FF ↔ #E0F4FF (white→#E0F4FF bg)
 * Then 5-pixel gradient chaser that alternates light blue ↔ black each lap
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
static const char* MQTT_HOST = "mqtt.cetools.org";
static const int   MQTT_PORT = 1884;  
static const char* MQTT_CLIENTID = "MKR1010_Cubo_FaceEffects";

#ifdef MQTT_USERNAME
  #define HAS_MQTT_AUTH 1
#else
  #define HAS_MQTT_AUTH 0
#endif

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

/********* Topics *********/
const char* mqtt_cmd_topic         = "student/CASA0014/luminaire/cmd";
const char* mqtt_base_topic        = "student/CASA0014/luminaire";
const char* user_update_topic      = "student/CASA0014/luminaire/user";
const char* brightness_update_topic= "student/CASA0014/luminaire/brightness";

/********* User & local-mirror *********/
int  LUMINAIRE_USER       = 25;   // change if needed
int  LUMINAIRE_BRIGHTNESS = 150;  // 0..255 (local mirror only)
char mqtt_data_topic[64];         // student/CASA0014/luminaire/<user>

/********* Website “virtual strip” *********/
#define NEOPIXEL_COUNT 72
#define NEOPIXEL_DATA_LENGTH (NEOPIXEL_COUNT * 3)
uint8_t frameBuf[NEOPIXEL_DATA_LENGTH];

/********* Optional local mirror strip *********/
#define NEOPIXEL_PIN 6
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

/********* IMU *********/
MPU6050 mpu;  // default address 0x68

/********* FACE NORMALS *********/
const float N6[6][3] = {
  { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 }
};
const char* N6name[6] = { "+X", "-X", "+Y", "-Y", "+Z", "-Z" };

const float sN = 0.57735026919f;  // 1/sqrt(3)
const float N8[8][3] = {
  { sN, sN, sN }, { sN, sN, -sN }, { sN, -sN, sN }, { sN, -sN, -sN },
  { -sN, sN, sN }, { -sN, sN, -sN }, { -sN, -sN, sN }, { -sN, -sN, -sN }
};
const char* N8name[8] = {
  "+ + +", "+ + -", "+ - +", "+ - -",
  "- + +", "- + -", "- - +", "- - -"
};

/********* Detection thresholds & timing *********/
const float ENTER_TH   = 0.80f;
//entry threshold for detecting a face as ‘down’
//arccos(0.80) ≈ 36.9°

const float EXIT_TH    = 0.72f;
//exit threshold for leaving that face
//arccos(0.72) ≈ 44.0°

//So a face is conunted as ‘down’ if its normal is within 36.9 degrees
//It stays down until you tilt it past 44 degrees

// ENTER_TH < score < EXIT_TH creates a 7.1 degree hysteresis band that prevents flicker/rapid toggling when you are near the boundary


const float LPF_ALPHA  = 0.20f;
//this is the smoothing factor for the exponential 
//moving average(EMA) on the gravity vector

const unsigned long DWELL_MS       = 10000UL;  // 10 s confirm
const unsigned long REARM_DROP_MS  = 800UL;

struct FaceState {
  bool  isSquare;
  int   idx;
  float score;
};

static inline float invSqrt(float x) { return 1.0f / sqrtf(x); }
static inline bool sameFace(const FaceState& a, const FaceState& b) {
  return (a.idx == b.idx) && (a.isSquare == b.isSquare);
}

/*** Detection state ***/
FaceState currentFace   = { true, -1, -1.0f };
bool      haveLPF       = false;
float     gx_f=0, gy_f=0, gz_f=0;

FaceState dwellCandidate= { true, -1, -1.0f };
unsigned long dwellStartMs = 0;

FaceState lastConfirmed = { true, -1, -1.0f };

unsigned long belowStartMs = 0;
bool belowActive = false;

/********* Effect program *********/
const unsigned long DURATION_SQUARE_MS   = 20000UL; // 20 seconds
const unsigned long DURATION_TRIANGLE_MS = 10000UL; // 10 seconds

const unsigned long SCAN_PERIOD_MS  = 150UL; // every 150 ms advance row/column
const unsigned long CHASE_PERIOD_MS = 80UL;  // every 80 ms advance chaser

bool programRunning = false, programIsSquare = false;
unsigned long programStart = 0, effectStartMs = 0;

unsigned long animStartTriangle = 0;

// button control
const int BTN_PIN = 7;          // D7 connected to DFR0785-G "D"
const unsigned long DEBOUNCE_MS = 30;
const bool BTN_ACTIVE_HIGH = true;   // if your reading is inverted, flip to false

volatile bool detectEnabled = true;
volatile bool g_logFrameOnce = false;   // reset at loop start; first publish prints

unsigned long btnLastChange = 0;
bool btnPrev = LOW;             // module idles LOW (not pressed)

//-----------
enum EffectMode { EFFECT_IDLE, EFFECT_SQUARE, EFFECT_TRIANGLE, EFFECT_YELLOW };
EffectMode effect = EFFECT_IDLE;

bool yellowLatched = false; 

// variables
int  scanRow  = 0;   // 0..5
int  scanCol  = 0;   // 0..11
int  chaseIdx = 0;   // 0..71
unsigned long lastStepMs = 0;

/*** Temporal gradient blink state ***/
float rowBlinkT = 0.0f; int rowBlinkDir = 1;
float colBlinkT = 0.0f; int colBlinkDir = 1;
const float BLINK_DELTA = 0.12f;

/********* Onboard RGB helpers (via NINA) — 25=R, 26=G, 27=B *********/
static inline void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  WiFiDrv::pinMode(25, OUTPUT);
  WiFiDrv::pinMode(26, OUTPUT);
  WiFiDrv::pinMode(27, OUTPUT);
  WiFiDrv::analogWrite(25, r);
  WiFiDrv::analogWrite(26, g);
  WiFiDrv::analogWrite(27, b);
}
static inline void LedRed()    { setRGB(155, 0, 0); }
static inline void LedGreen()  { setRGB(0, 155, 0); }
static inline void LedBlue()   { setRGB(0, 0, 155); }
static inline void LedYellow() { setRGB(255, 255, 0); }
static inline void LedWhite()  { setRGB(255, 255, 255); }

/********* Color + FX helpers *********/
struct Color { uint8_t r, g, b; };
const Color WHITE = { 255, 255, 255 };

// Row gradient
const Color GRAD_ROW_A = { 0xFC, 0x99, 0xFF };  // #FC99FF
const Color GRAD_ROW_B = { 0xFF, 0xA7, 0x4F };  // #FFA74F

const Color GRAD_BG_A = {0xF8, 0x9B, 0x29}; // #F89B29 (top) 
const Color GRAD_BG_B = {0xE7, 0x61, 0xBD}; // #E761BD (bottom)

// Column gradient (not used in progressive, kept for compat)
const Color GRAD_COL_A = { 0xCA, 0xD0, 0xFF };  // #CAD0FF
const Color GRAD_COL_B = { 0xE0, 0xF4, 0xFF };  // #E0F4FF

// Chaser endpoints
const Color GRAD_CHASE_B = { 0xF8, 0xFF, 0xBD }; // #f8ffbd
const Color GRAD_CHASE_A = { 0x00, 0x00, 0x00 };  // black

// gradient for triangle surface (L→R)
const Color BG_LR_A = {0x83, 0xF5, 0xE5}; // #83F5E5 
const Color BG_LR_B = {0xE7, 0x61, 0xBD}; // #E761BD

class FX {
public:
  static inline Color lerp(const Color& a, const Color& b, float t) {
    if (t < 0) t = 0; if (t > 1) t = 1;
    Color c;
    c.r = (uint8_t)(a.r + (b.r - a.r) * t);
    c.g = (uint8_t)(a.g + (b.g - a.g) * t);
    c.b = (uint8_t)(a.b + (b.b - a.b) * t);
    return c;
  }

  static inline void fillSolid(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < NEOPIXEL_COUNT; i++) {
      frameBuf[i * 3 + 0] = r;
      frameBuf[i * 3 + 1] = g;
      frameBuf[i * 3 + 2] = b;
    }
  }
  

  static inline int idxFromRowCol(int r, int c) { return c * 6 + r; }

  // Row blink-through: background = ROW gradient (WHITE->bgTarget); active row = lerp(a,b,t)
  static void rowBlinkThrough(int r, const Color& a, const Color& b, const Color& bgTarget, float t) {
    if (r < 0) r = 0;
    if (r > 5) r = 5;

    // Fill background: each row has a single uniform color
    for (int rr = 0; rr < 6; rr++) {
      float tr = (6 > 1) ? (float)rr / 5.0f : 0.0f;   // 0..1 by row
      Color cc = lerp(WHITE, bgTarget, tr);
      for (int c = 0; c < 12; c++) {
        int i = idxFromRowCol(rr, c);
        frameBuf[i * 3 + 0] = cc.r;
        frameBuf[i * 3 + 1] = cc.g;
        frameBuf[i * 3 + 2] = cc.b;
      }
    }

    // Override active row with the blink color, uniformly across the row
    Color rc = lerp(a, b, t);
    for (int c = 0; c < 12; c++) {
      int i = idxFromRowCol(r, c);
      frameBuf[i * 3 + 0] = rc.r;
      frameBuf[i * 3 + 1] = rc.g;
      frameBuf[i * 3 + 2] = rc.b;
    }
  }

  // Progressive background fill (top → bottom):
  // - Rows [0 .. rowsToFill-1] show the top→bottom gradient color for that row.
  // - Rows [rowsToFill .. 5] remain WHITE (not yet revealed).
  // Then overlay the active blink row 'r'.
  static void rowBlinkThroughProgressive(int r, const Color& a, const Color& b, float t, int rowsToFill) {
    if (r < 0) r = 0;
    if (r > 5) r = 5;
    if (rowsToFill < 0) rowsToFill = 0;
    if (rowsToFill > 6) rowsToFill = 6;

    for (int rr = 0; rr < 6; rr++) {
      // Compute the per-row gradient factor (top=0 → bottom=1)
      float tr = (6 > 1) ? (float)rr / 5.0f : 0.0f;

      // If this row is already "revealed", use its gradient color; else keep white
      Color rowColor = (rr < rowsToFill) ? lerp(GRAD_BG_A, GRAD_BG_B, tr) : WHITE;

      // Paint the whole row with that color
      for (int c = 0; c < 12; c++) {
        int i = idxFromRowCol(rr, c);
        frameBuf[i * 3 + 0] = rowColor.r;
        frameBuf[i * 3 + 1] = rowColor.g;
        frameBuf[i * 3 + 2] = rowColor.b;
      }
    }

    // Overlay active row with the blink color (uniform across the row)
    Color rc = lerp(a, b, t);
    for (int c = 0; c < 12; c++) {
      int i = idxFromRowCol(r, c);
      frameBuf[i * 3 + 0] = rc.r;
      frameBuf[i * 3 + 1] = rc.g;
      frameBuf[i * 3 + 2] = rc.b;
    }
  }



  // X-pixel gradient chaser window; black elsewhere
  static void chaserWindow(int startIdx, int window, const Color& a, const Color& b) {
    fillSolid(0, 0, 0);
    if (window < 1) window = 1;
    for (int k = 0; k < window; k++) {
      int i = (startIdx + k) % NEOPIXEL_COUNT;
      float t = (window > 1) ? (float)k / (float)(window - 1) : 0.0f;
      Color cc = lerp(a, b, t);
      frameBuf[i * 3 + 0] = cc.r;
      frameBuf[i * 3 + 1] = cc.g;
      frameBuf[i * 3 + 2] = cc.b;
    }
  }

  static inline Color toGray(const Color& c) {
    uint8_t y = (uint8_t)((0.2126f * c.r) + (0.7152f * c.g) + (0.0722f * c.b) + 0.5f);
    return { y, y, y };
  }

  // Colored stripe uses L→R palette (#83F5E5 → #E761BD).
  // Unrevealed columns = greyscale; revealed = color. Background black.
  static void veeredSinglePerColGradientProgressive(int startCol, int windowCols,
                                                    int baseRow, int veer,
                                                    const Color& /*centerFallback*/,
                                                    const Color& haloTarget,   // pass BLACK
                                                    int radius,
                                                    int colsToReveal) {
    if (windowCols < 1) windowCols = 1;
    if (radius < 0) radius = 0;
    if (colsToReveal < 0) colsToReveal = 0;
    if (colsToReveal > 12) colsToReveal = 12;

    // Background black
    fillSolid(0, 0, 0);

    // Draw moving diagonal, column by column
    for (int k = 0; k < windowCols; ++k) {
      const int c = (startCol + k) % 12;                       // 0..11
      const int centerR = ((baseRow + k * veer) % 6 + 6) % 6;  // 0..5

      float tc = (12 > 1) ? (float)c / 11.0f : 0.0f;
      Color colCenterBase = lerp(BG_LR_A, BG_LR_B, tc);
      bool revealed = (c < colsToReveal);

      Color colCenter = revealed ? colCenterBase : toGray(colCenterBase);

      // Center pixel
      {
        int i = idxFromRowCol(centerR, c);
        frameBuf[i * 3 + 0] = colCenter.r;
        frameBuf[i * 3 + 1] = colCenter.g;
        frameBuf[i * 3 + 2] = colCenter.b;
      }

      // Vertical halo within same column
      if (radius > 0) {
        for (int d = 1; d <= radius; ++d) {
          float tHalo = (float)d / (float)radius;                  // 0→1
          Color haloBase = lerp(colCenterBase, haloTarget, tHalo); // fade to black
          Color halo     = revealed ? haloBase : toGray(haloBase);

          int rUp = centerR - d;
          if (rUp >= 0) {
            int iUp = idxFromRowCol(rUp, c);
            frameBuf[iUp * 3 + 0] = halo.r;
            frameBuf[iUp * 3 + 1] = halo.g;
            frameBuf[iUp * 3 + 2] = halo.b;
          }
          int rDn = centerR + d;
          if (rDn <= 5) {
            int iDn = idxFromRowCol(rDn, c);
            frameBuf[iDn * 3 + 0] = halo.r;
            frameBuf[iDn * 3 + 1] = halo.g;
            frameBuf[iDn * 3 + 2] = halo.b;
          }
        }
      }
    }
  }
}; // end class FX

/********* Publish / mirror *********/
bool publishFrame() {
  if (!mqttClient.connected()) {
    if (g_logFrameOnce) { Serial.println("[MQTT] Not connected; frame NOT published."); g_logFrameOnce = false; }
    return false;
  }
  bool ok = mqttClient.publish(mqtt_data_topic, frameBuf, NEOPIXEL_DATA_LENGTH);
  if (g_logFrameOnce) {
    Serial.print("[PUB] ");
    Serial.print(NEOPIXEL_DATA_LENGTH);
    Serial.print("B -> ");
    Serial.println(mqtt_data_topic);
    if (!ok) { Serial.print("  state="); Serial.println(mqttClient.state()); }
    g_logFrameOnce = false;     // print only once this loop()
  }
  return ok;
}

void showLocalMirror() {
  pixels.setBrightness(LUMINAIRE_BRIGHTNESS);
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    pixels.setPixelColor(i,
      frameBuf[i * 3 + 0], frameBuf[i * 3 + 1], frameBuf[i * 3 + 2]);
  }
  pixels.show();
}
void publishSolidAndShow(uint8_t r, uint8_t g, uint8_t b) {
  FX::fillSolid(r, g, b);
  publishFrame();
  setRGB(r, g, b);
  showLocalMirror();
}

// centralize disable behavior
void disableDetection() {
  detectEnabled = false;
  dwellCandidate.idx = -1;
  currentFace.idx = -1;
  effect = EFFECT_IDLE;
  publishSolidAndShow(0, 0, 0);
  Serial.println("[BTN] detection DISABLED");
}
void enableDetection() {
  detectEnabled = true;
  Serial.println("[BTN] detection ENABLED");
}

/********* MQTT status JSON (optional telemetry) *********/
void publishCmdJSON(const char* palette, uint16_t seconds, const char* faceName) {
  char buf[200];
  snprintf(buf, sizeof(buf),
           "{\"device\":\"%s\",\"cmd\":\"timer\",\"palette\":\"%s\",\"seconds\":%u,\"face\":\"%s\"}",
           MQTT_CLIENTID, palette, seconds, faceName ? faceName : "");
  mqttClient.publish(mqtt_cmd_topic, buf);
}
void publishState(const char* faceName, const char* faceType, float score, bool valid, bool locked) {
  char buf[220];
  snprintf(buf, sizeof(buf),
           "{\"device\":\"%s\",\"cmd\":\"state\",\"face\":\"%s\",\"type\":\"%s\",\"score\":%.3f,\"valid\":%s,\"locked\":%s}",
           MQTT_CLIENTID, faceName ? faceName : "none", faceType ? faceType : "none",
           score, valid ? "true" : "false", locked ? "true" : "false");
  mqttClient.publish(mqtt_cmd_topic, buf);
}

/********* Wi-Fi *********/
void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi module not present!");
    return;
  }
  LedBlue();
  Serial.print("Connecting WiFi: ");
  Serial.println(ssid);
  int st = WL_IDLE_STATUS;
  while (st != WL_CONNECTED) {
    st = WiFi.begin(ssid, pass);
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi OK, IP: ");
  Serial.println(WiFi.localIP());
  LedGreen();
}

/********* MQTT *********/
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, user_update_topic) == 0) {
    char s[16];
    unsigned n = min(length, (unsigned)(sizeof(s) - 1));
    memcpy(s, payload, n);
    s[n] = '\0';
    int new_user = atoi(s);
    if (new_user != LUMINAIRE_USER) {
      // unsubscribe old
      mqttClient.unsubscribe(mqtt_data_topic);



      // subscribe new
      snprintf(mqtt_data_topic, sizeof(mqtt_data_topic), "%s/%d", mqtt_base_topic, LUMINAIRE_USER);
      mqttClient.subscribe(mqtt_data_topic);

      Serial.print("[TOPIC] -> ");
      Serial.println(mqtt_data_topic);
      publishSolidAndShow(0, 0, 0);  // optional clear
    }
    return;
  }
  if (strcmp(topic, brightness_update_topic) == 0) {
    char s[16];
    unsigned n = min(length, (unsigned)(sizeof(s) - 1));
    memcpy(s, payload, n);
    s[n] = '\0';
    int nb = atoi(s);
    if (nb < 0) nb = 0;
    if (nb > 255) nb = 255;
    LUMINAIRE_BRIGHTNESS = nb;
    showLocalMirror();
    Serial.print("BRIGHTNESS -> ");
    Serial.println(LUMINAIRE_BRIGHTNESS);
    return;
  }
}
unsigned long lastConnectionAttempt = 0;
const unsigned long RECONNECT_INTERVAL_MS = 5000;
void ensureMQTT() {
  if (mqttClient.connected()) return;
  if (millis() - lastConnectionAttempt < RECONNECT_INTERVAL_MS) return;
  lastConnectionAttempt = millis();

  LedBlue();
  Serial.print("MQTT connect ");
  Serial.print(MQTT_HOST);
  Serial.print(":");
  Serial.println(MQTT_PORT);
  bool ok = false;
#if HAS_MQTT_AUTH
  ok = mqttClient.connect(MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD);
#else
  ok = mqttClient.connect(MQTT_CLIENTID);
#endif
  if (ok) {
    mqttClient.subscribe(user_update_topic);
    mqttClient.subscribe(brightness_update_topic);
    snprintf(mqtt_data_topic, sizeof(mqtt_data_topic), "%s/%d", mqtt_base_topic, LUMINAIRE_USER);
    mqttClient.subscribe(mqtt_data_topic);
    Serial.print("[TOPIC] Website/frames: ");
    Serial.println(mqtt_data_topic);
    LedGreen();
    publishSolidAndShow(0, 0, 0);
  } else {
    Serial.print("MQTT fail rc=");
    Serial.println(mqttClient.state());
    LedRed();
  }
}

/********* EFFECT ENGINE *********/
unsigned long programDurationMs() {
  return programIsSquare ? DURATION_SQUARE_MS : DURATION_TRIANGLE_MS;
}
bool chaseFlip = false;

inline void latchYellowOn() {
  // Ensure detection is running while showing yellow
  detectEnabled = true;

  // NEW: reconnect WiFi + MQTT when (re)enabling detection
  reconnectWiFiAndMQTT();   

  // Start yellow fresh
  effect = EFFECT_YELLOW;
  chaseIdx  = 0;
  lastStepMs = 0;
  chaseFlip = false;
  effectStartMs = millis();

  yellowLatched = true;

  publishCmdJSON("manual_chaser", 0, "button-toggle-on");
}


inline void latchYellowOff() {
    // Stop effects and stop detection
    effect = EFFECT_IDLE;
    publishSolidAndShow(0, 0, 0);    // clear website/local mirror

    // reset dwell state so it won't immediately retrigger
    dwellCandidate.idx = -1;
    currentFace.idx = -1;

    detectEnabled = false;
    yellowLatched = false;

    // Optional indicators:
    // setRGB(0,155,0);                // or LedGreen()/LedYellow()
    // setButtonLed(false);            // if you wired a button LED to a GPIO
    publishCmdJSON("manual_chaser", 0, "button-toggle-off");
  }

inline void reconnectWiFiAndMQTT() {
  // Make sure WiFi is up
  ensureWiFi();

  // If MQTT is down, connect; if it's already up, (re)subscribe to be safe
  if (!mqttClient.connected()) {
    ensureMQTT();
  } else {
    // refresh subscriptions (and data topic in case user changed)
    mqttClient.unsubscribe(mqtt_data_topic);
    snprintf(mqtt_data_topic, sizeof(mqtt_data_topic), "%s/%d", mqtt_base_topic, LUMINAIRE_USER);
    mqttClient.subscribe(mqtt_data_topic);
    mqttClient.subscribe(user_update_topic);
    mqttClient.subscribe(brightness_update_topic);
  }
}


void updateEffects() {
  unsigned long now = millis(), dur = programDurationMs();
  switch (effect) {
    case EFFECT_IDLE: break;

    case EFFECT_SQUARE: {
      // Row blink-through for 20s
      if (now - effectStartMs >= dur) {
        effect = EFFECT_YELLOW;
        chaseIdx = 0;
        lastStepMs = 0;
        publishCmdJSON("grad_chaser", 0, "square");
        break;
      }

      if (now - lastStepMs >= SCAN_PERIOD_MS) {
        lastStepMs = now;

        // --- How many rows should show the background gradient so far? (0..6)
        const uint32_t stepMs = DURATION_SQUARE_MS / 7UL;
        uint32_t elapsed = now - effectStartMs;
        int rowsToFill = (elapsed >= DURATION_SQUARE_MS) ? 7 : (int)(elapsed / stepMs);
        rowsToFill = rowsToFill < 0 ? 0 : (rowsToFill > 7 ? 7 : rowsToFill);

        // --- draw + publish (top→bottom progressive background)
        FX::rowBlinkThroughProgressive(scanRow, GRAD_ROW_A, GRAD_ROW_B, 0.5f, rowsToFill);

        publishFrame();
        setRGB(GRAD_ROW_A.r, GRAD_ROW_A.g, GRAD_ROW_A.b);
        showLocalMirror();

        // advance temporal gradient and move to next row
        rowBlinkT += BLINK_DELTA * rowBlinkDir;
        if (rowBlinkT >= 1.0f) { rowBlinkT = 1.0f; rowBlinkDir = -1; }
        if (rowBlinkT <= 0.0f) { rowBlinkT = 0.0f; rowBlinkDir = 1; }
        scanRow = (scanRow + 1) % 6;
      }
    } break;


    case EFFECT_TRIANGLE: {
      if (now - effectStartMs >= dur) {
        effect = EFFECT_YELLOW;
        chaseIdx = 0;
        lastStepMs = 0;
        publishCmdJSON("grad_chaser", 0, "triangle");
        break;
      }

      if (now - lastStepMs >= SCAN_PERIOD_MS) {
        lastStepMs = now;

        // motion: slide 1 column every 100 ms, drift row every 6 steps
        const uint16_t COL_PERIOD_MS = 100;
        const uint8_t  DRIFT_EVERY   = 6;
        const int      WINDOW_COLS   = 12;  // one pixel in every column
        const int      VEER          = 1;   // +1 = down-right, use -1 to flip

        unsigned long steps = (now - animStartTriangle) / COL_PERIOD_MS;
        int startCol = steps % WINDOW_COLS;
        int baseRow  = (steps / DRIFT_EVERY) % 6;

        // 13 slices: 0..12 (0 = all greyscale)
        const uint32_t stepMs = DURATION_TRIANGLE_MS / 13UL;
        uint32_t elapsed = now - effectStartMs;
        int colsToReveal = (elapsed >= DURATION_TRIANGLE_MS) ? 12 : (int)(elapsed / stepMs);
        if (colsToReveal < 0)  colsToReveal = 0;
        if (colsToReveal > 12) colsToReveal = 12;

        // Draw (halo toward black so it blends to background)
        FX::veeredSinglePerColGradientProgressive(
          startCol, WINDOW_COLS, baseRow, VEER,
          /*centerFallback*/ GRAD_ROW_A,
          /*haloTarget*/ Color{0,0,0},
          /*radius=*/4,
          colsToReveal
        );

        publishFrame();
        showLocalMirror();
      }
    } break;

    case EFFECT_YELLOW: {
      if (now - lastStepMs >= CHASE_PERIOD_MS) {
        lastStepMs = now;
        const Color cA = chaseFlip ? GRAD_CHASE_B : GRAD_CHASE_A;
        const Color cB = chaseFlip ? GRAD_CHASE_A : GRAD_CHASE_B;

        FX::chaserWindow(chaseIdx, 5, cA, cB);
        publishFrame();
        setRGB(cB.r, cB.g, cB.b);
        showLocalMirror();

        chaseIdx = (chaseIdx + 1) % NEOPIXEL_COUNT;
        if (chaseIdx == 0) chaseFlip = !chaseFlip;
      }
    } break;
  }
}

/********* SETUP *********/
void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();       
  while (!Serial && millis() - t0 < 2000) {}

  pinMode(BTN_PIN, INPUT); 
  btnPrev = (digitalRead(BTN_PIN) == (BTN_ACTIVE_HIGH ? HIGH : LOW));  // seed prev state

  // Local mirror
  pixels.begin();
  pixels.setBrightness(LUMINAIRE_BRIGHTNESS);
  pixels.show();

  // MQTT setup
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqtt_callback);
  mqttClient.setBufferSize(512);

  snprintf(mqtt_data_topic, sizeof(mqtt_data_topic), "%s/%d", mqtt_base_topic, LUMINAIRE_USER);
  Serial.print("[TOPIC] Website/frames: ");
  Serial.println(mqtt_data_topic);

  ensureWiFi();
  ensureMQTT();

  // IMU
  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection FAIL");
    while (1) {
      setRGB(155, 0, 0);
      delay(400);
      setRGB(0, 0, 155);
      delay(400);
    }
  }

  setRGB(0, 155, 0);
  Serial.println("Hold a face steady for 10 s to confirm. Same face is ignored next time.");
}

/********* LOOP *********/
void loop() {
  ensureWiFi();
  ensureMQTT();
  mqttClient.loop();
  g_logFrameOnce = true;

  bool raw = digitalRead(BTN_PIN);
  bool btnNow = (raw == (BTN_ACTIVE_HIGH ? HIGH : LOW));
  // button control

  unsigned long now = millis();

  // debounce
  if (btnNow != btnPrev && (now - btnLastChange) > DEBOUNCE_MS) {
    btnLastChange = now;
    btnPrev = btnNow;
    if (btnNow) {             // on press
      if (!yellowLatched) latchYellowOn();
      else                latchYellowOff();
    }
  }

  // ---- EFFECTS ALWAYS RUN ----
  updateEffects();

  if (!detectEnabled) { delay(10); return; }

  // ---- IMU READ ----
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  //When the accelerometer is configured at ±2 g range, 
  //its sensitivity is 16 384 Least Significant Bit per g. 
  //Dividing the raw readings (ax, ay, az) by 16384.0f
  //converts them from raw counts to g units 
  //(so a stationary device might read ≈ (0, 0, 1)).🦜



  float gx = ax / 16384.0f, gy = ay / 16384.0f, gz = az / 16384.0f;

  //Following is a safety check.
  //If the squared length is near zero, it means the vector 
  //is almost zero — maybe a bad reading, disconnected sensor, 
  //or free-fall (no gravity detected).
  //Instead of dividing by zero (which would produce NaN), 
  //it just waits a bit and skips the update.


  float n2 = gx * gx + gy * gy + gz * gz;
  if (n2 < 1e-6f) { delay(10); return; }

  float invn = invSqrt(n2);
  gx *= invn; gy *= invn; gz *= invn;

  // now ||(gx,gy,gz)|| = 1, 
  //After this, (gx, gy, gz) is a unit gravity direction vector.

  // Low-pass + renormalize
  // Low-pass = smoothing: It removes high-frequency 
  // jitter/noise and short bumps from the accelerometer 
  // so the “which face is down” logic doesn’t flicker.
  // LPF_ALPHA is the smoothing factor.


  // Low-pass + renormalize
  if (!haveLPF) {
    gx_f = gx; gy_f = gy; gz_f = gz; haveLPF = true;
  } else {
    // 1) Low-pass (EMA / IIR(1))
    gx_f = (1.0f - LPF_ALPHA) * gx_f + LPF_ALPHA * gx;
    gy_f = (1.0f - LPF_ALPHA) * gy_f + LPF_ALPHA * gy;
    gz_f = (1.0f - LPF_ALPHA) * gz_f + LPF_ALPHA * gz;

    // 2) Re-normalize to unit length
    float m2 = gx_f * gx_f + gy_f * gy_f + gz_f * gz_f;
    float invm = invSqrt(m2);
    gx_f *= invm; gy_f *= invm; gz_f *= invm;
  }

  // Best face = argmax -dot(n, g)
  float bestScore = -1e9f; int bestIdx = -1; bool bestIsSquare = true;

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
      currentFace.idx = bestIdx;
      currentFace.score = bestScore;
    }
  } else {
    if (bestIsSquare == currentFace.isSquare && bestIdx == currentFace.idx) {
      currentFace.score = bestScore;
    } else {
      if (bestScore > ENTER_TH || currentFace.score < EXIT_TH) {
        currentFace.isSquare = bestIsSquare;
        currentFace.idx = bestIdx;
        currentFace.score = bestScore;
        belowActive = false;
      }
    }
  }

  bool haveCandidate = (currentFace.idx != -1) && (currentFace.score > ENTER_TH);

  // Suppress SAME face as last confirmed
  if (haveCandidate && lastConfirmed.idx != -1 && sameFace(currentFace, lastConfirmed)) {
    dwellCandidate.idx = -1;
    delay(10);
    return;
  }

  // dip re-arm (robustness)
  if (lastConfirmed.idx != -1) {
    if (currentFace.score < EXIT_TH) {
      if (!belowActive) { belowActive = true; belowStartMs = millis(); }
      else if (millis() - belowStartMs >= REARM_DROP_MS) { belowActive = false; }
    } else {
      belowActive = false;
    }
  }

  if (!haveCandidate) { dwellCandidate.idx = -1; delay(10); return; }

  unsigned long nowMs = millis();

  // Dwell accumulation on new different face
  if (dwellCandidate.idx == -1 || !sameFace(dwellCandidate, currentFace)) {
    dwellCandidate = currentFace;
    dwellStartMs   = nowMs;
  } else {
    unsigned long elapsed = nowMs - dwellStartMs;
    if (elapsed >= DWELL_MS) {
      const char* faceType = currentFace.isSquare ? "square" : "triangle";
      const char* faceName = currentFace.isSquare ? N6name[currentFace.idx] : N8name[currentFace.idx];
      Serial.print("[CONFIRMED 10s] ");
      Serial.print(faceType);
      Serial.print(" DOWN: ");
      Serial.print(faceName);
      Serial.print("   score=");
      Serial.println(currentFace.score, 3);
      publishState(faceName, faceType, currentFace.score, true, true);

      // Start the program for this face
      programIsSquare = currentFace.isSquare;
      programRunning  = true;
      programStart    = nowMs;
      lastStepMs      = 0;
      rowBlinkT = colBlinkT = 0.0f;
      rowBlinkDir = colBlinkDir = 1;

      if (programIsSquare) {
        scanRow = 0;
        effect = EFFECT_SQUARE;
        effectStartMs = nowMs;
        publishCmdJSON("row_temporal_grad", DURATION_SQUARE_MS / 1000, faceName);
      } else {
        scanCol = 0;
        effect = EFFECT_TRIANGLE;
        effectStartMs = nowMs;
        animStartTriangle = nowMs;  // set anim clock once on entry
        publishCmdJSON("col_temporal_grad", DURATION_TRIANGLE_MS / 1000, faceName);
      }

      lastConfirmed = currentFace;
      dwellCandidate.idx = -1;  // re-arm for different face
    }
  }

  delay(10);
}

