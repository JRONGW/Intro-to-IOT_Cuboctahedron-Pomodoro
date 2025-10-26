/***************************************************************
 * MKR1010 — 72×RGB publisher for website/Vespera (RETained)
 * 
 * Publishes 216-byte frames (R,G,B per LED × 72 LEDs) to:
 *   student/CASA0014/luminaire/<USER>
 * using MQTT on port 1884 (TCP), retained = true.
 ***************************************************************/
#include <SPI.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <utility/wifi_drv.h>   // onboard RGB helper (25=R,26=G,27=B)
#include "arduino_secrets.h"    // SECRET_SSID / SECRET_PASS, optional MQTT_USERNAME/MQTT_PASSWORD

/********* USER/TOPIC *********/
static const int   LUMINAIRE_USER  = 25;  // <- change if needed
static const char* TOPIC_BASE      = "student/CASA0014/luminaire";
static char        TOPIC[64];             // student/CASA0014/luminaire/25

/********* WIFI *********/
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

/********* MQTT *********/
static const char* MQTT_HOST   = "mqtt.cetools.org";
static const int   MQTT_PORT   = 1884;          // <<— changed to 1884
static const char* MQTT_CLIENT = "MKR1010_Website_Frame_Publisher";

// Optional creds from arduino_secrets.h. If not defined, connect without auth.
#ifdef MQTT_USERNAME
char mqtt_user[] = MQTT_USERNAME;
char mqtt_pass[] = MQTT_PASSWORD;
#else
char mqtt_user[] = "";
char mqtt_pass[] = "";
#endif

WiFiClient wifi;
PubSubClient mqtt(wifi);

/********* FRAME *********/
static const int NUM_LEDS    = 72;
static const int FRAME_BYTES = NUM_LEDS * 3;     // 216
uint8_t frameBuf[FRAME_BYTES];

/********* TIMERS *********/
unsigned long lastSend      = 0;
const unsigned long SEND_MS = 800;

/********* ONBOARD RGB *********/
static inline void ledRGB(uint8_t r,uint8_t g,uint8_t b){
  WiFiDrv::analogWrite(25, r);  // R
  WiFiDrv::analogWrite(26, g);  // G
  WiFiDrv::analogWrite(27, b);  // B
}
static inline void ledRed()   { ledRGB(180,0,0); }
static inline void ledGreen() { ledRGB(0,180,0); }
static inline void ledBlue()  { ledRGB(0,0,180); }
static inline void ledOff()   { ledRGB(0,0,0);   }

/********* UTILS: fill + print *********/
void fillSolid(uint8_t r, uint8_t g, uint8_t b){
  for (int i=0; i<NUM_LEDS; i++){
    frameBuf[i*3+0] = r;
    frameBuf[i*3+1] = g;
    frameBuf[i*3+2] = b;
  }
}

void printPreviewAndHex(const char* tag){
  Serial.print(tag); Serial.println();
  Serial.print("  first 12 LEDs: ");
  for (int i=0; i<12; i++){
    Serial.print("(");
    Serial.print(frameBuf[i*3+0]); Serial.print(",");
    Serial.print(frameBuf[i*3+1]); Serial.print(",");
    Serial.print(frameBuf[i*3+2]); Serial.print(") ");
  }
  Serial.println();
  Serial.print("  hexdump (first 48 bytes): ");
  for (int i=0;i<48;i++){
    uint8_t v = frameBuf[i];
    if (v < 16) Serial.print('0');
    Serial.print(v, HEX);
    Serial.print((i%16==15) ? "  " : " ");
  }
  Serial.println();
}

// Publish the current frame, **retained**, and print result
bool publishFrame(){
  bool ok = mqtt.publish(TOPIC, frameBuf, FRAME_BYTES, true /* RETAINED */);
  Serial.print("PUB "); Serial.print(FRAME_BYTES); Serial.print("B ");
  Serial.print(ok ? "OK" : "FAIL");
  Serial.print(" -> "); Serial.println(TOPIC);
  return ok;
}

/********* WIFI/MQTT CONNECT *********/
void ensureWiFi(){
  if (WiFi.status() == WL_CONNECTED) return;
  ledBlue();
  Serial.print("Connecting WiFi: "); Serial.println(ssid);
  int st = WL_IDLE_STATUS;
  while (st != WL_CONNECTED){
    st = WiFi.begin(ssid, pass);
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("WiFi OK, IP: "); Serial.println(WiFi.localIP());
  ledGreen();
}

unsigned long lastMqttAttempt = 0;
const unsigned long RECONNECT_MS = 2000;

void ensureMQTT(){
  if (mqtt.connected()) return;
  if (millis() - lastMqttAttempt < RECONNECT_MS) return;
  lastMqttAttempt = millis();

  ledBlue();
  Serial.print("MQTT connect… (");
  Serial.print(MQTT_HOST); Serial.print(":"); Serial.print(MQTT_PORT); Serial.println(")");
  bool ok;
  if (mqtt_user[0]) {
    ok = mqtt.connect(MQTT_CLIENT, mqtt_user, mqtt_pass);
  } else {
    ok = mqtt.connect(MQTT_CLIENT);
  }
  if (ok){
    Serial.println("connected.");
    ledGreen();

    // Boot burst so subscribers immediately see a retained frame
    fillSolid(255,0,0);    printPreviewAndHex("boot: RED");    publishFrame(); delay(250);
    fillSolid(0,255,0);    printPreviewAndHex("boot: GREEN");  publishFrame(); delay(250);
    fillSolid(0,0,255);    printPreviewAndHex("boot: BLUE");   publishFrame(); delay(250);
    fillSolid(255,255,255);printPreviewAndHex("boot: WHITE");  publishFrame(); delay(250);
    fillSolid(0,0,0);      printPreviewAndHex("boot: BLACK");  publishFrame();
  } else {
    Serial.print("failed, rc="); Serial.println(mqtt.state());
    ledRed();
  }
}

/********* SETUP *********/
void setup(){
  Serial.begin(115200);
  while(!Serial) {}

  // Onboard RGB pins
  WiFiDrv::pinMode(25, OUTPUT);
  WiFiDrv::pinMode(26, OUTPUT);
  WiFiDrv::pinMode(27, OUTPUT);
  ledRed();

  // Compose topic: student/CASA0014/luminaire/<USER>
  snprintf(TOPIC, sizeof(TOPIC), "%s/%d", TOPIC_BASE, LUMINAIRE_USER);
  Serial.print("Publishing to topic: "); Serial.println(TOPIC);

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(512);  // enough for 216 bytes
  mqtt.setKeepAlive(30);

  ensureWiFi();
  ensureMQTT();
}

/********* LOOP *********/
int stepIdx = 0;

void loop(){
  ensureWiFi();
  ensureMQTT();
  mqtt.loop();

  // cycle colors every 800 ms
  unsigned long now = millis();
  if (now - lastSend >= SEND_MS && mqtt.connected()){
    lastSend = now;

    switch (stepIdx % 5){
      case 0: fillSolid(0,   0, 255); ledRGB(0,0,120);     printPreviewAndHex("BLUE");  publishFrame(); break;
      case 1: fillSolid(255,255,255); ledRGB(120,120,120); printPreviewAndHex("WHITE"); publishFrame(); break;
      case 2: fillSolid(0,   0,   0); ledRGB(0,0,0);       printPreviewAndHex("BLACK"); publishFrame(); break;
      case 3: fillSolid(255, 0,   0); ledRGB(120,0,0);     printPreviewAndHex("RED");   publishFrame(); break;
      case 4: fillSolid(0, 255,   0); ledRGB(0,120,0);     printPreviewAndHex("GREEN"); publishFrame(); break;
    }
    stepIdx++;
  }
}
