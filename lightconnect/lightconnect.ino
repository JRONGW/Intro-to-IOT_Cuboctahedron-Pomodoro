/********* INCLUDES *********/
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"   // Rowberg DMP header

#include <SPI.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <utility/wifi_drv.h>

#include "arduino_secrets.h" // defines SECRET_SSID / SECRET_PASS

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/********* WIFI / MQTT *********/
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

// MQTT broker + topics
const char* mqtt_server     = "mqtt.cetools.org";
const int   mqtt_port       = 1883;
const char* mqtt_client_id  = "MKR1010_NeoPixel_Luminaire_Vespera";
const char* mqtt_cmd_topic  = "student/CASA0014/luminaire/cmd";   // we PUBLISH timer + state here

const char* mqtt_base_topic = "student/CASA0014/luminaire";       // for per-user frame payloads
char        mqtt_data_topic[64];                                   // will be "<base>/<LUMINAIRE_USER>"

const char* user_update_topic      = "student/CASA0014/luminaire/user";
const char* brightness_update_topic= "student/CASA0014/luminaire/brightness";

// App state that can be updated via MQTT
int LUMINAIRE_USER       = 25;   // <— set your default user id here
int LUMINAIRE_BRIGHTNESS = 150;

/********* NEOPIXEL *********/
#define NEOPIXEL_PIN   6
#define NEOPIXEL_COUNT 72
#define NEOPIXEL_DATA_LENGTH (NEOPIXEL_COUNT * 3)

WiFiClient       wifiClient;
PubSubClient     mqttClient(wifiClient);
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Optional: local preview of the running program on the strip
const bool LOCAL_PREVIEW_ON_NEOPIXEL = false;

/********* IMU / DMP *********/
MPU6050  mpu(0x68);
bool     dmpReady   = false;
uint16_t packetSize = 0;
uint16_t fifoCount  = 0;
uint8_t  fifoBuffer[64];

Quaternion  q;
VectorFloat gravity;
float       ypr[3];

/********* FACE LOGIC *********/
// Lock after N ms steady
const unsigned long STABLE_REQUIRED_MS = 10000UL; // 10 s

// Program durations
const unsigned long DURATION_SQUARE_MS   = 30UL * 1000UL; // 30 s
const unsigned long DURATION_TRIANGLE_MS = 10UL * 1000UL; // 10 s
const unsigned long BLINK_PERIOD_MS      = 1500UL;

// Angle thresholds (deg) based on your experiments (with hysteresis)
const float THR_SQUARE_ENTER_DEG = 25.0f;
const float THR_SQUARE_EXIT_DEG  = 30.0f;

const float THR_TRI_ENTER_DEG    = 20.0f;
const float THR_TRI_EXIT_DEG     = 25.0f;

struct Normal { float x,y,z; bool isSquare; const char* name; };
Normal SQUARES[6] = {
  {  1,  0,  0, true,  "+X" },
  { -1,  0,  0, true,  "-X" },
  {  0,  1,  0, true,  "+Y" },
  {  0, -1,  0, true,  "-Y" },
  {  0,  0,  1, true,  "+Z" },
  {  0,  0, -1, true,  "-Z" }
};
const float N = 0.57735027f;
Normal TRIS[8] = {
  {  N,  N,  N, false, "+++" },
  {  N,  N, -N, false, "++-" },
  {  N, -N,  N, false, "+-+" },
  {  N, -N, -N, false, "+--" },
  { -N,  N,  N, false, "-++" },
  { -N,  N, -N, false, "-+-" },
  { -N, -N,  N, false, "--+" },
  { -N, -N, -N, false, "---" }
};
Normal ALL[14];

static inline float dot3(float ax,float ay,float az,float bx,float by,float bz){
  return ax*bx + ay*by + az*bz;
}

// face state
int currentFace = -1, lastFace = -1;
int lastReportedFace = -2; // for printing candidate changes
bool faceLocked = false;
unsigned long faceChangeTime = 0;

// program state
bool programRunning   = false;
bool programIsSquare  = false;
unsigned long programStart = 0;

/********* RECONNECT CONTROL *********/
unsigned long lastConnectionAttempt = 0;
const unsigned long RECONNECT_INTERVAL_MS = 5000;

/********* HELPERS *********/
void buildNormals(){
  for (int i=0;i<6;i++)   ALL[i]   = SQUARES[i];
  for (int i=0;i<8;i++)   ALL[6+i] = TRIS[i];
}

void LedRed(){   WiFiDrv::digitalWrite(25, LOW); WiFiDrv::digitalWrite(26, HIGH); WiFiDrv::digitalWrite(27, LOW); }
void LedGreen(){ WiFiDrv::digitalWrite(25, HIGH);WiFiDrv::digitalWrite(26, LOW);  WiFiDrv::digitalWrite(27, LOW); }
void LedBlue(){  WiFiDrv::digitalWrite(25, LOW); WiFiDrv::digitalWrite(26, LOW);  WiFiDrv::digitalWrite(27, HIGH); }

void showAll(uint8_t r,uint8_t g,uint8_t b){
  for (int i=0;i<NEOPIXEL_COUNT;i++) pixels.setPixelColor(i, r,g,b);
  pixels.show();
}

/********* MQTT STATE PUBLISH *********/
void publishCmdJSON(const char* palette, uint16_t seconds, const char* faceName){
  char buf[200];
  snprintf(buf, sizeof(buf),
    "{\"device\":\"%s\",\"cmd\":\"timer\",\"palette\":\"%s\",\"seconds\":%u,\"face\":\"%s\"}",
    mqtt_client_id, palette, seconds, faceName ? faceName : "");
  mqttClient.publish(mqtt_cmd_topic, buf);
}

void publishState(const char* faceName, const char* faceType, float angleDeg, float dot, bool valid, bool locked){
  // minimal JSON to the same cmd topic with cmd:"state"
  char buf[240];
  snprintf(buf, sizeof(buf),
    "{\"device\":\"%s\",\"cmd\":\"state\",\"face\":\"%s\",\"type\":\"%s\",\"angle\":%.1f,\"dot\":%.3f,\"valid\":%s,\"locked\":%s}",
    mqtt_client_id,
    faceName ? faceName : "none",
    faceType ? faceType : "none",
    angleDeg, dot,
    valid ? "true" : "false",
    locked ? "true" : "false"
  );
  mqttClient.publish(mqtt_cmd_topic, buf);
}

/********* WIFI / MQTT *********/
void ensureWiFi(){
  if (WiFi.status() == WL_CONNECTED) return;

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi module not present!");
    while (true) delay(1000);
  }

  LedBlue();
  Serial.print("Connecting to WiFi: "); Serial.println(ssid);

  int st = WL_IDLE_STATUS;
  while (st != WL_CONNECTED){
    st = WiFi.begin(ssid, pass);
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  LedGreen();
}

void mqtt_callback(char* topic, byte* payload, unsigned int length){
  // Copy payload into a null-terminated string
  char payload_str[length+1];
  memcpy(payload_str, payload, length);
  payload_str[length] = '\0';

  if (strcmp(topic, user_update_topic) == 0){
    int new_user = atoi(payload_str);
    if (new_user != LUMINAIRE_USER){
      // Unsub old topic
      mqttClient.unsubscribe(mqtt_data_topic);
      LUMINAIRE_USER = new_user;
      snprintf(mqtt_data_topic, sizeof(mqtt_data_topic), "%s/%d", mqtt_base_topic, LUMINAIRE_USER);
      mqttClient.subscribe(mqtt_data_topic);
      Serial.print("LUMINAIRE_USER -> "); Serial.println(LUMINAIRE_USER);

      pixels.clear(); pixels.show();
    }
  } else if (strcmp(topic, brightness_update_topic) == 0){
    int b = atoi(payload_str);
    LUMINAIRE_BRIGHTNESS = b;
    pixels.setBrightness(LUMINAIRE_BRIGHTNESS);
    Serial.print("LUMINAIRE_BRIGHTNESS -> "); Serial.println(LUMINAIRE_BRIGHTNESS);
  } else {
    // Per-user frame topic: update NeoPixels from binary RGB payload
    // (length must equal NEOPIXEL_DATA_LENGTH)
    if (length == NEOPIXEL_DATA_LENGTH){
      for (int i=0;i<NEOPIXEL_COUNT;i++){
        byte r = payload[i*3 + 0];
        byte g = payload[i*3 + 1];
        byte b = payload[i*3 + 2];
        pixels.setPixelColor(i, r,g,b);
      }
      pixels.show();
    }
  }
}

void ensureMQTT(){
  if (mqttClient.connected()) return;
  if (millis() - lastConnectionAttempt < RECONNECT_INTERVAL_MS) return;
  lastConnectionAttempt = millis();

  LedBlue();
  Serial.print("MQTT connecting… ");
  if (mqttClient.connect(mqtt_client_id)){
    Serial.println("connected");

    bool ok = true;
    ok &= mqttClient.subscribe(user_update_topic);
    ok &= mqttClient.subscribe(brightness_update_topic);
    ok &= mqttClient.subscribe(mqtt_data_topic);
    if (ok) LedGreen(); else LedRed();
  } else {
    Serial.print("failed, rc="); Serial.println(mqttClient.state());
    LedRed();
  }
}

/********* IMU *********/
void imuSetup(){
  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();
  if (!mpu.testConnection()){
    Serial.println("MPU6050 connection FAIL");
    while(1);
  }
  int st = mpu.dmpInitialize();
  if (st != 0){
    Serial.print("DMP init fail: "); Serial.println(st);
    while(1);
  }
  // Optional: some library versions have this; if it errors, keep it commented
  // mpu.dmpSetFIFORate(50);

  mpu.setDMPEnabled(true);

  // Polling mode (INT optional)
  mpu.setInterruptMode(false);
  mpu.setInterruptDrive(false);
  mpu.setInterruptLatch(true);
  mpu.setInterruptLatchClear(true);
  mpu.setIntEnabled(0x02);

  packetSize = mpu.dmpGetFIFOPacketSize();
  dmpReady   = true;

  buildNormals();
  faceChangeTime = millis();

  Serial.print("DMP ready. packetSize="); Serial.println(packetSize);
}

/********* PROGRAM PREVIEW (optional) *********/
void runProgramPreview(){
  if (!LOCAL_PREVIEW_ON_NEOPIXEL || !programRunning) return;

  unsigned long now = millis();
  unsigned long dur = programIsSquare ? DURATION_SQUARE_MS : DURATION_TRIANGLE_MS;
  unsigned long el  = now - programStart;
  if (el >= dur){ programRunning = false; return; }

  float t = (float)el / (float)dur; if (t<0) t=0; if (t>1) t=1;

  uint8_t r=0,g=0,b=0;
  if (programIsSquare){ r=255; g=(uint8_t)(255*t); b=0; }           // red → yellow
  else                 { r=0;   g=(uint8_t)(255*t); b=(uint8_t)(255*(1-t)); } // blue → green

  float phase = (now % BLINK_PERIOD_MS) / (float)BLINK_PERIOD_MS;
  float env   = (phase < 0.5f) ? (phase*2.0f) : (2.0f - phase*2.0f);
  float scale = 0.25f + 0.75f * env;

  showAll((uint8_t)(r*scale),(uint8_t)(g*scale),(uint8_t)(b*scale));
}

/********* ARDUINO SETUP *********/
void setup(){
  Serial.begin(115200);
  while(!Serial){}

  // onboard RGB
  WiFiDrv::pinMode(25, OUTPUT); // G
  WiFiDrv::pinMode(26, OUTPUT); // R
  WiFiDrv::pinMode(27, OUTPUT); // B
  LedRed();

  // neopixels
  pixels.begin();
  pixels.show();
  pixels.setBrightness(LUMINAIRE_BRIGHTNESS);

  // MQTT config
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqtt_callback);

  // per-user data topic
  snprintf(mqtt_data_topic, sizeof(mqtt_data_topic), "%s/%d", mqtt_base_topic, LUMINAIRE_USER);
  Serial.print("Data topic: "); Serial.println(mqtt_data_topic);

  // WiFi + MQTT
  ensureWiFi();
  ensureMQTT();

  // IMU
  imuSetup();

  LedGreen();
}

/********* ARDUINO LOOP *********/
void loop(){
  ensureWiFi();
  ensureMQTT();
  mqttClient.loop();

  if (!dmpReady) { runProgramPreview(); return; }

  fifoCount = mpu.getFIFOCount();

  if (fifoCount == 1024){
    mpu.resetFIFO();
    Serial.println("FIFO overflow -> reset");
    return;
  }
  if (fifoCount < packetSize){
    delay(2);
    runProgramPreview();
    return;
  }

  while (fifoCount >= packetSize){
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // Parse orientation
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Serial ypr (for your reference)
    static uint32_t lastYPR = 0;
    uint32_t nowms = millis();
    if (nowms - lastYPR >= 50){
      Serial.print("ypr\t");
      Serial.print(ypr[0]*180.0/M_PI); Serial.print('\t');
      Serial.print(ypr[1]*180.0/M_PI); Serial.print('\t');
      Serial.println(ypr[2]*180.0/M_PI);
      lastYPR = nowms;
    }

    // Down vector (unit) is -gravity
    float ux = -gravity.x;
    float uy = -gravity.y;
    float uz = -gravity.z;

    // Find best face
    int   bestIdx = -1;
    float bestDot = -2.0f;
    for (int i=0;i<14;i++){
      float d = dot3(ux,uy,uz, ALL[i].x,ALL[i].y,ALL[i].z);
      if (d > bestDot){ bestDot = d; bestIdx = i; }
    }

    // Convert bestDot to angle
    float angleDeg = 0.0f;
    if (bestIdx >= 0){
      float clamped = bestDot;
      if (clamped >  1.0f) clamped = 1.0f;
      if (clamped < -1.0f) clamped = -1.0f;
      angleDeg = acosf(clamped) * 180.0f / M_PI;
    }

    // Candidate validity using ENTER thresholds
    bool candidateValid = false;
    if (bestIdx >= 0){
      if (ALL[bestIdx].isSquare) candidateValid = (angleDeg <= THR_SQUARE_ENTER_DEG);
      else                       candidateValid = (angleDeg <= THR_TRI_ENTER_DEG);
    }
    int detectedFace = candidateValid ? bestIdx : -1;

    // Report candidate changes (prints + MQTT state)
    if (bestIdx != lastReportedFace){
      lastReportedFace = bestIdx;
      if (bestIdx >= 0){
        const char* ftype = ALL[bestIdx].isSquare ? "square" : "triangle";
        Serial.print("DOWN candidate: "); Serial.print(ALL[bestIdx].name);
        Serial.print("  type=");  Serial.print(ftype);
        Serial.print("  angle="); Serial.print(angleDeg, 1); Serial.print("°");
        Serial.print("  dot=");   Serial.print(bestDot, 3);
        Serial.print("  valid="); Serial.println(candidateValid ? "yes" : "no");
        publishState(ALL[bestIdx].name, ftype, angleDeg, bestDot, candidateValid, faceLocked);
      } else {
        Serial.println("DOWN candidate: none  type=none");
        publishState("none", "none", 0.0f, -2.0f, false, faceLocked);
      }
    }

    // Stability + trigger logic
    static unsigned long faceChangeLocal = millis();
    unsigned long now = millis();

    if (detectedFace != lastFace){
      lastFace = detectedFace;
      faceChangeLocal = now;
      // cancel running program on movement
      if (programRunning){
        programRunning = false;
      }
      faceLocked = false;
    } else {
      // Try to lock after steady time
      if (!faceLocked && detectedFace >= 0 && (now - faceChangeLocal >= STABLE_REQUIRED_MS)){
        faceLocked  = true;
        currentFace = detectedFace;

        programIsSquare = ALL[currentFace].isSquare;
        programRunning  = true;
        programStart    = now;

        const char* ftype = programIsSquare ? "square" : "triangle";
        Serial.print("LOCKED "); Serial.print(ftype);
        Serial.print(" face ");   Serial.print(ALL[currentFace].name);
        Serial.print("  angle="); Serial.print(angleDeg, 1); Serial.println("°");

        publishState(ALL[currentFace].name, ftype, angleDeg, bestDot, true, true);

        if (programIsSquare){
          publishCmdJSON("red_yellow",  DURATION_SQUARE_MS/1000,   ALL[currentFace].name);
        } else {
          publishCmdJSON("blue_green", DURATION_TRIANGLE_MS/1000, ALL[currentFace].name);
        }
      }
    }

    // If locked, apply EXIT thresholds to decide when to unlock
    if (faceLocked && currentFace >= 0){
      const bool lockedIsSquare = ALL[currentFace].isSquare;
      float exitDeg = lockedIsSquare ? THR_SQUARE_EXIT_DEG : THR_TRI_EXIT_DEG;

      float d = dot3(ux,uy,uz, ALL[currentFace].x,ALL[currentFace].y,ALL[currentFace].z);
      if (d >  1.0f) d = 1.0f;
      if (d < -1.0f) d = -1.0f;
      float lockedAngleDeg = acosf(d) * 180.0f / M_PI;

      if (lockedAngleDeg >= exitDeg){
        faceLocked = false;
        programRunning = false;
        Serial.println("Unlocked: angle exceeded EXIT threshold");
        publishState(ALL[currentFace].name,
                     lockedIsSquare ? "square" : "triangle",
                     lockedAngleDeg, d, false, false);
      }
    }
  }

  runProgramPreview(); // optional NeoPixel preview
}
