/***************************************************************
 * MKR WiFi 1010 + MPU-6050 (GY-521) — Cuboctahedron Face Detection
 * Website/Vespera: publishes 72xRGB frames to student/CASA0014/luminaire/<USER>
 * Triangle DOWN -> 10 s blink BLUE <-> WHITE, then YELLOW hold
 * Square   DOWN -> 20 s blink PINK <-> WHITE, then YELLOW hold
 * Onboard RGB mirrors colors (25=R, 26=G, 27=B)
 ***************************************************************/
#include <math.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#include <SPI.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <utility/wifi_drv.h>

#include "arduino_secrets.h"  // SECRET_SSID / SECRET_PASS / (optional) MQTT_USERNAME / MQTT_PASSWORD

/********* WIFI / MQTT *********/
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

const char* mqtt_server = "mqtt.cetools.org";
const int   mqtt_port   = 1884; // <-- matches your Feather sketch
const char* mqtt_client_id = "MKR1010_Vespera_Publisher";
const char* mqtt_cmd_topic  = "student/CASA0014/luminaire/cmd";
const char* mqtt_base_topic = "student/CASA0014/luminaire";
char        mqtt_data_topic[64]; // student/CASA0014/luminaire/<LUMINAIRE_USER>

#ifdef MQTT_USERNAME
char mqtt_user[] = MQTT_USERNAME;
char mqtt_pass[] = MQTT_PASSWORD;
#endif

const char* user_update_topic       = "student/CASA0014/luminaire/user";
const char* brightness_update_topic = "student/CASA0014/luminaire/brightness";

int LUMINAIRE_USER       = 25;
int LUMINAIRE_BRIGHTNESS = 150;

/********* “Virtual strip” for website (72 pixels) *********/
#define NEOPIXEL_COUNT 72
#define NEOPIXEL_DATA_LENGTH (NEOPIXEL_COUNT * 3)

// Optional local mirror; safe even with no strip connected
#define NEOPIXEL_PIN 6
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

/********* IMU *********/
MPU6050 mpu;

/********* FACE NORMALS *********/
struct Normal { float x,y,z; bool isSquare; const char* name; };
Normal SQUARES[6] = {
  { 1, 0, 0, true, "+X" }, { -1, 0, 0, true, "-X" },
  { 0, 1, 0, true, "+Y" }, {  0,-1, 0, true, "-Y" },
  { 0, 0, 1, true, "+Z" }, {  0, 0,-1, true, "-Z" }
};
const float N = 0.57735027f;
Normal TRIS[8] = {
  {  N,  N,  N, false, "+++" }, {  N,  N, -N, false, "++-" },
  {  N, -N,  N, false, "+-+" }, {  N, -N, -N, false, "+--" },
  { -N,  N,  N, false, "-++" }, { -N,  N, -N, false, "-+-" },
  { -N, -N,  N, false, "--+" }, { -N, -N, -N, false, "---" }
};
Normal ALL[14];
static inline float dot3(float ax,float ay,float az,float bx,float by,float bz){ return ax*bx + ay*by + az*bz; }

/********* DETECTION PARAMS *********/
const unsigned long STABLE_REQUIRED_MS = 10000UL;
const float ENTER_TH  = 0.80f;
const float EXIT_TH   = 0.72f;
const float LPF_ALPHA = 0.20f;
const unsigned long REARM_DROP_MS = 800UL;

struct FaceScore { int idx; bool isSquare; float score; };
FaceScore currentFace = { -1, true, -1.0f };
FaceScore dwellCandidate = { -1, true, -1.0f };
unsigned long dwellStartMs = 0;
bool announcedThisFace = false;

bool haveLPF=false; float gx_f=0, gy_f=0, gz_f=0;
unsigned long belowStartMs=0; bool belowActive=false;
bool postYellowLock=false; int lockedFaceIdx=-1; bool lockedFaceIsSquare=true;
unsigned long lockBelowStartMs=0; bool lockBelowActive=false;
int  lastConfirmedIdx=-1; bool lastConfirmedIsSquare=true;

/********* PROGRAM / EFFECTS *********/
const unsigned long DURATION_SQUARE_MS   = 20000UL;
const unsigned long DURATION_TRIANGLE_MS = 10000UL;
const unsigned long BLINK_PERIOD_MS      = 300UL;

bool programRunning=false, programIsSquare=false;
unsigned long programStart=0, effectStartMs=0;
bool publishedYellowAtEnd=false;

enum EffectMode { EFFECT_IDLE, EFFECT_SQUARE, EFFECT_TRIANGLE };
EffectMode effect = EFFECT_IDLE;

/********* MQTT + WiFi *********/
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

/********* Onboard RGB helpers (PWM via NINA) *********/
static inline void setRGB(uint8_t r,uint8_t g,uint8_t b){
  WiFiDrv::analogWrite(25, r);
  WiFiDrv::analogWrite(26, g);
  WiFiDrv::analogWrite(27, b);
}
void LedRed(){    setRGB(155,0,0); }
void LedGreen(){  setRGB(0,155,0); }
void LedBlue(){   setRGB(0,0,155); }
void LedYellow(){ setRGB(255,255,0); }
void LedWhite(){  setRGB(255,255,255); }
void LedPink(){   setRGB(255,64,128); }

/********* WEBSITE FRAME PUBLISHING *********/
uint8_t desicredR=0, desiredG=0, desiredB=0;
uint8_t frameBuf[NEOPIXEL_DATA_LENGTH];

void setDesiredColor(uint8_t r,uint8_t g,uint8_t b){
  desiredR=r; desiredG=g; desiredB=b;
  // mirror onboard + (optional) local strip immediately
  setRGB(r,g,b);
  pixels.setBrightness(LUMINAIRE_BRIGHTNESS);
  for (int i=0;i<NEOPIXEL_COUNT;i++) pixels.setPixelColor(i, r,g,b);
  pixels.show();
}

void fillSolidFrameFromDesired(){
  for (int i=0;i<NEOPIXEL_COUNT;i++){
    frameBuf[i*3 + 0] = desiredR;
    frameBuf[i*3 + 1] = desiredG;
    frameBuf[i*3 + 2] = desiredB;
  }
}

bool publishFrame(){
  if (!mqttClient.connected()){
    // Serial.println("[MQTT] Not connected; frame NOT published.");
    return false;
  }
  fillSolidFrameFromDesired();
  // binary payload for website/Vespera (72 * RGB)
  bool ok = mqttClient.publish(mqtt_data_topic, frameBuf, NEOPIXEL_DATA_LENGTH);
  // Serial.print("[PUB] "); Serial.print(mqtt_data_topic); Serial.print(" -> "); Serial.println(ok?"OK":"FAIL");
  return ok;
}

const unsigned long WEBSITE_PUSH_MS = 200;  // faster heartbeat
unsigned long lastWebsitePush = 0;

/********* MQTT JSON STATUS (optional) *********/
void publishCmdJSON(const char* palette, uint16_t seconds, const char* faceName){
  char buf[200];
  snprintf(buf, sizeof(buf),
    "{\"device\":\"%s\",\"cmd\":\"timer\",\"palette\":\"%s\",\"seconds\":%u,\"face\":\"%s\"}",
    mqtt_client_id, palette, seconds, faceName ? faceName : "");
  mqttClient.publish(mqtt_cmd_topic, buf);
}
void publishState(const char* faceName, const char* faceType, float score, bool valid, bool locked){
  char buf[220];
  snprintf(buf, sizeof(buf),
    "{\"device\":\"%s\",\"cmd\":\"state\",\"face\":\"%s\",\"type\":\"%s\",\"score\":%.3f,\"valid\":%s,\"locked\":%s}",
    mqtt_client_id, faceName ? faceName : "none", faceType ? faceType : "none",
    score, valid ? "true":"false", locked ? "true":"false");
  mqttClient.publish(mqtt_cmd_topic, buf);
}

/********* WIFI / MQTT *********/
void ensureWiFi(){
  if (WiFi.status() == WL_CONNECTED) return;
  if (WiFi.status() == WL_NO_MODULE) { Serial.println("WiFi module not present!"); return; }
  LedBlue();
  Serial.print("Connecting WiFi: "); Serial.println(ssid);
  int st = WL_IDLE_STATUS;
  while (st != WL_CONNECTED){ st = WiFi.begin(ssid, pass); delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());
  LedGreen();
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // user id update
  if (strcmp(topic, user_update_topic) == 0){
    char s[16]; unsigned n = (length < sizeof(s)-1) ? length : (sizeof(s)-1); memcpy(s,payload,n); s[n]='\0';
    int new_user = atoi(s);
    if (new_user != LUMINAIRE_USER){
      mqttClient.unsubscribe(mqtt_data_topic);
      LUMINAIRE_USER = new_user;
      snprintf(mqtt_data_topic, sizeof(mqtt_data_topic), "%s/%d", mqtt_base_topic, LUMINAIRE_USER);
      mqttClient.subscribe(mqtt_data_topic);
      setDesiredColor(0,0,0);
      publishFrame(); // push a clear frame to the new topic
      Serial.print("Switched publish topic -> "); Serial.println(mqtt_data_topic);
    }
    return;
  }
  // brightness update
  if (strcmp(topic, brightness_update_topic) == 0){
    char s[16]; unsigned n = (length < sizeof(s)-1) ? length : (sizeof(s)-1); memcpy(s,payload,n); s[n]='\0';
    int nb = atoi(s); if (nb<0) nb=0; if (nb>255) nb=255;
    LUMINAIRE_BRIGHTNESS = nb;
    pixels.setBrightness(LUMINAIRE_BRIGHTNESS);
    pixels.show();
    Serial.print("LUMINAIRE_BRIGHTNESS -> "); Serial.println(LUMINAIRE_BRIGHTNESS);
    return;
  }
}

unsigned long lastConnectionAttempt=0;
const unsigned long RECONNECT_INTERVAL_MS=3000;
void ensureMQTT(){
  if (mqttClient.connected()) return;
  if (millis() - lastConnectionAttempt < RECONNECT_INTERVAL_MS) return;
  lastConnectionAttempt = millis();

  LedBlue();
  Serial.print("MQTT connecting… ");
#ifdef MQTT_USERNAME
  if (mqttClient.connect(mqtt_client_id, mqtt_user, mqtt_pass))
#else
  if (mqttClient.connect(mqtt_client_id))
#endif
  {
    Serial.println("connected");
    mqttClient.subscribe(user_update_topic);
    mqttClient.subscribe(brightness_update_topic);
    snprintf(mqtt_data_topic, sizeof(mqtt_data_topic), "%s/%d", mqtt_base_topic, LUMINAIRE_USER);
    mqttClient.subscribe(mqtt_data_topic);
    Serial.print("[TOPIC] Website/frames topic: "); Serial.println(mqtt_data_topic);
    LedGreen();

    // quick boot test on the website
    setDesiredColor(255,0,0); publishFrame(); delay(200);
    setDesiredColor(0,255,0); publishFrame(); delay(200);
    setDesiredColor(0,0,255); publishFrame(); delay(200);
    setDesiredColor(0,0,0);   publishFrame();
  } else {
    Serial.print("failed, rc="); Serial.println(mqttClient.state());
    LedRed();
  }
}

/********* OPTIONAL SENSOR->BODY ROTATION *********/
void applyBodyRotation(float& x,float& y,float& z){ /* identity */ }

/********* NORMALS BUILD *********/
Normal ALLtmp[14];
void buildNormals(){ for (int i=0;i<6;i++) ALL[i]=SQUARES[i]; for (int i=0;i<8;i++) ALL[6+i]=TRIS[i]; }

/********* TIMER PRINT STATE *********/
int lastPrintedRemainSec = -1;
unsigned long programDurationMs(){ return programIsSquare ? DURATION_SQUARE_MS : DURATION_TRIANGLE_MS; }
void maybePrintCountdown(){
  if (!programRunning){ lastPrintedRemainSec=-1; return; }
  unsigned long now=millis(), elapsed=now-programStart, dur=programDurationMs();
  if (elapsed>dur) elapsed=dur;
  int remainSec = (int)((dur - elapsed + 999)/1000);
  if (remainSec != lastPrintedRemainSec){
    lastPrintedRemainSec = remainSec;
    Serial.print("[TIMER] "); Serial.print(programIsSquare ? "square":"triangle");
    Serial.print(" remaining: "); Serial.print(remainSec); Serial.println(" s");
  }
}

void rearmDetection(){
  announcedThisFace=false;
  dwellCandidate.idx=-1;
  belowActive=false;
  lastPrintedRemainSec=-1;
}

/********* EFFECTS -> set desired color only; website frames pushed on a heartbeat *********/
void updateEffects(){
  unsigned long now=millis(), dur=programDurationMs();
  static bool lastBlinkPhase = false;

  switch (effect){
    case EFFECT_IDLE:
      break;

    case EFFECT_SQUARE: { // PINK <-> WHITE for 20s
      unsigned long elapsed = now - effectStartMs;
      if (elapsed >= dur){
        setDesiredColor(255,255,0); // YELLOW hold
        if (!publishedYellowAtEnd){ publishCmdJSON("yellow",0,"square"); publishedYellowAtEnd=true; }
        effect = EFFECT_IDLE; programRunning=false;
        postYellowLock=true; lockedFaceIdx=lastConfirmedIdx; lockedFaceIsSquare=lastConfirmedIsSquare; lockBelowActive=false;
        rearmDetection();
        break;
      }
      bool phase = ((elapsed / BLINK_PERIOD_MS) % 2) == 0;
      if (phase != lastBlinkPhase){
        lastBlinkPhase = phase;
        if (phase) setDesiredColor(255,64,128); else setDesiredColor(255,255,255);
      }
    } break;

    case EFFECT_TRIANGLE: { // BLUE <-> WHITE for 10s
      unsigned long elapsed = now - effectStartMs;
      if (elapsed >= dur){
        setDesiredColor(255,255,0); // YELLOW hold
        if (!publishedYellowAtEnd){ publishCmdJSON("yellow",0,"triangle"); publishedYellowAtEnd=true; }
        effect = EFFECT_IDLE; programRunning=false;
        postYellowLock=true; lockedFaceIdx=lastConfirmedIdx; lockedFaceIsSquare=lastConfirmedIsSquare; lockBelowActive=false;
        rearmDetection();
        break;
      }
      bool phase = ((elapsed / BLINK_PERIOD_MS) % 2) == 0;
      if (phase != lastBlinkPhase){
        lastBlinkPhase = phase;
        if (phase) setDesiredColor(0,0,255); else setDesiredColor(255,255,255);
      }
    } break;
  }
  maybePrintCountdown();
}

/********* SETUP *********/
void setup(){
  Serial.begin(115200);
  while(!Serial){}

  // Onboard RGB pins
  WiFiDrv::pinMode(25, OUTPUT);
  WiFiDrv::pinMode(26, OUTPUT);
  WiFiDrv::pinMode(27, OUTPUT);
  LedRed();

  // Optional local mirror
  pixels.begin();
  pixels.setBrightness(LUMINAIRE_BRIGHTNESS);
  pixels.show();

  // MQTT — set buffer/keepalive BEFORE first connect so 216-byte frames fit reliably
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqtt_callback);
  mqttClient.setBufferSize(512);
  mqttClient.setKeepAlive(30);

  // topics
  snprintf(mqtt_data_topic, sizeof(mqtt_data_topic), "%s/%d", mqtt_base_topic, LUMINAIRE_USER);
  Serial.print("[TOPIC] Website/frames topic: "); Serial.println(mqtt_data_topic);

  ensureWiFi();
  ensureMQTT();

  // IMU
  Wire.begin(); Wire.setClock(400000);
  mpu.initialize();
  if (!mpu.testConnection()){
    Serial.println("MPU6050 connection FAIL");
    while(1){ LedRed(); delay(500); LedBlue(); delay(500); }
  }

  // normals
  buildNormals();

  setDesiredColor(0,0,0);
  Serial.println("Hold a face steady for 10 s to confirm and start timer.");
}

/********* LOOP *********/
void loop(){
  ensureWiFi();
  ensureMQTT();
  mqttClient.loop();

  // ---- Periodically push current desired color to WEBSITE (every 200 ms) ----
  unsigned long now = millis();
  if (mqttClient.connected() && (now - lastWebsitePush >= WEBSITE_PUSH_MS)){
    lastWebsitePush = now;
    publishFrame();
  }

  // ---- IMU, face detection, and effects ----
  int16_t ax, ay, az; mpu.getAcceleration(&ax,&ay,&az);
  float gx=ax/16384.0f, gy=ay/16384.0f, gz=az/16384.0f;

  float n2=gx*gx+gy*gy+gz*gz; if (n2<1e-6f){ updateEffects(); delay(10); return; }
  float invn=1.0f/sqrtf(n2); gx*=invn; gy*=invn; gz*=invn;

  // LPF
  if (!haveLPF){ gx_f=gx; gy_f=gy; gz_f=gz; haveLPF=true; }
  else {
    gx_f=(1.0f-LPF_ALPHA)*gx_f + LPF_ALPHA*gx;
    gy_f=(1.0f-LPF_ALPHA)*gy_f + LPF_ALPHA*gy;
    gz_f=(1.0f-LPF_ALPHA)*gz_f + LPF_ALPHA*gz;
    float m2=gx_f*gx_f+gy_f*gy_f+gz_f*gz_f; float invm=1.0f/sqrtf(m2);
    gx_f*=invm; gy_f*=invm; gz_f*=invm;
  }

  // down vector
  float ux=-gx_f, uy=-gy_f, uz=-gz_f;

  // best face by dot
  int bestIdx=-1; float bestDot=-2.0f; bool bestIsSquare=true;
  for (int i=0;i<14;i++){
    float d=dot3(ux,uy,uz, ALL[i].x,ALL[i].y,ALL[i].z);
    if (d>bestDot){ bestDot=d; bestIdx=i; bestIsSquare=ALL[i].isSquare; }
  }

  bool candidateValid = (bestIdx>=0) && (bestDot>=ENTER_TH);

  // hysteresis tracking
  if (currentFace.idx==-1){
    if (candidateValid){ currentFace.idx=bestIdx; currentFace.isSquare=bestIsSquare; currentFace.score=bestDot; }
  } else {
    if (bestIdx==currentFace.idx){ currentFace.score=bestDot; }
    else if (bestDot>ENTER_TH || currentFace.score<EXIT_TH){
      currentFace.idx=bestIdx; currentFace.isSquare=bestIsSquare; currentFace.score=bestDot; belowActive=false;
    }
  }

  // Post-yellow lock
  if (postYellowLock){
    if (currentFace.idx>=0 && currentFace.isSquare==lockedFaceIsSquare && currentFace.idx==lockedFaceIdx){
      if (currentFace.score<EXIT_TH){
        if (!lockBelowActive){ lockBelowActive=true; lockBelowStartMs=millis(); }
        else if (millis()-lockBelowStartMs>=REARM_DROP_MS){
          postYellowLock=false; lockBelowActive=false; announcedThisFace=false; dwellCandidate.idx=-1;
        }
      } else { lockBelowActive=false; }
    } else { postYellowLock=false; lockBelowActive=false; announcedThisFace=false; dwellCandidate.idx=-1; }
  }

  // re-arm gating after announcement
  if (announcedThisFace){
    if (currentFace.score<EXIT_TH){
      if (!belowActive){ belowActive=true; belowStartMs=millis(); }
      else if (millis()-belowStartMs>=REARM_DROP_MS){
        announcedThisFace=false; dwellCandidate.idx=-1; belowActive=false;
      }
    } else { belowActive=false; }
  }

  // dwell confirmation
  if (!postYellowLock){
    bool haveCandidate = (currentFace.idx!=-1) && (currentFace.score>ENTER_TH);
    if (haveCandidate){
      if (dwellCandidate.idx==-1 || dwellCandidate.idx!=currentFace.idx){
        dwellCandidate=currentFace; dwellStartMs=now;
      } else {
        unsigned long elapsed = now - dwellStartMs;
        if (!announcedThisFace && elapsed>=STABLE_REQUIRED_MS){
          programIsSquare = dwellCandidate.isSquare;
          programRunning  = true;
          programStart    = now;
          publishedYellowAtEnd = false;

          lastConfirmedIdx = dwellCandidate.idx;
          lastConfirmedIsSquare = dwellCandidate.isSquare;

          if (programIsSquare){
            effect = EFFECT_SQUARE; effectStartMs=now;
            setDesiredColor(255,64,128);
            publishCmdJSON("pink_white_blink", DURATION_SQUARE_MS/1000, ALL[dwellCandidate.idx].name);
            Serial.print("[CONFIRMED] Square DOWN: "); Serial.println(ALL[dwellCandidate.idx].name);
          } else {
            effect = EFFECT_TRIANGLE; effectStartMs=now;
            setDesiredColor(0,0,255);
            publishCmdJSON("blue_white_blink", DURATION_TRIANGLE_MS/1000, ALL[dwellCandidate.idx].name);
            Serial.print("[CONFIRMED] Triangle DOWN: "); Serial.println(ALL[dwellCandidate.idx].name);
          }
          announcedThisFace=true;
          lastPrintedRemainSec=-1;
        }
      }
    } else { if (!announcedThisFace) dwellCandidate.idx=-1; }
  }

  // drive effects (only sets desired color; publisher pushes frames)
  updateEffects();

  delay(10);
}
