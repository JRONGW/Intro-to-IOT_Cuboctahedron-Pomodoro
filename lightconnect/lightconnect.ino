/********* INCLUDES *********/
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"

#include <SPI.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <utility/wifi_drv.h>

#include "arduino_secrets.h" // defines SECRET_SSID / SECRET_PASS

/********* WIFI / MQTT *********/
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the Wifi radio's status

// --- MQTT Broker Configuration ---
// Replace with your MQTT broker's IP address or hostname
const char* mqtt_server = "mqtt.cetools.org";
const int mqtt_port = 1883;                                // Default MQTT port (unsecured)
const char* mqtt_client_id = "MKR1010_NeoPixel_Luminaire_Vespera"; // Unique client ID for your device
const char* mqtt_cmd_topic = "student/CASA0014/luminaire/cmd";


const char* mqtt_base_topic = "student/CASA0014/luminaire";

// The full topic string for THIS device's updates (will be generated dynamically)
// Assuming user IDs are up to 4 digits, a length of 64 is safe.
char mqtt_data_topic[64];

// Fixed control topics
const char* user_update_topic = "student/CASA0014/luminaire/user";
const char* brightness_update_topic = "student/CASA0014/luminaire/brightness";


// --- Use this variable to see if we should change lights or not
// --- Defaults to 0 - staff user - on start up
int LUMINAIRE_USER = 0;
int LUMINAIRE_BRIGHTNESS = 150;


/********* NEOPIXEL *********/
#define NEOPIXEL_PIN   6
#define NEOPIXEL_COUNT 72
#define NEOPIXEL_DATA_LENGTH (NEOPIXEL_COUNT * 3) // 3 bytes per LED (R, G, B)

// --- Global Objects ---
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);



// Optional: show the running program locally on the strip (preview)
// If false, we only publish MQTT commands and still render any subscribed frames.
const bool LOCAL_PREVIEW_ON_NEOPIXEL = false;



/********* IMU / DMP *********/
MPU6050 mpu(0x68);
bool     dmpReady   = false;
uint16_t packetSize = 0;
uint16_t fifoCount  = 0;
uint8_t  fifoBuffer[64];

Quaternion  q;
VectorFloat gravity;
float       ypr[3];

/********* FACE LOGIC *********/
// Lock after 10 s steady
const unsigned long STABLE_REQUIRED_MS   = 10000UL;
// Program durations
const unsigned long DURATION_SQUARE_MS   = 30UL * 1000UL; // 30 s
const unsigned long DURATION_TRIANGLE_MS = 10UL * 1000UL; // 10 s
const unsigned long BLINK_PERIOD_MS      = 1500UL;

// Cosine thresholds (~20° for squares, ~15° for triangles)
const float COS_THR_SQUARE   = 0.9396926f; // cos(20°)
const float COS_THR_TRIANGLE = 0.9659258f; // cos(15°)

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
bool faceLocked = false;
unsigned long faceChangeTime = 0;

// program state
bool programRunning = false;
bool programIsSquare = false;
unsigned long programStart = 0;

// --- Connection Recovery Variables ---
unsigned long lastConnectionAttempt = 0;
const unsigned long RECONNECT_INTERVAL_MS = 5000; // Try reconnecting every 5 seconds

/********* HELPERS *********/
void buildNormals(){
  for (int i=0;i<6;i++)   ALL[i]   = SQUARES[i];
  for (int i=0;i<8;i++)   ALL[6+i] = TRIS[i];
}

void LedRed(){   WiFiDrv::digitalWrite(25, LOW); WiFiDrv::digitalWrite(26, HIGH); WiFiDrv::digitalWrite(27, LOW); }
void LedGreen(){ WiFiDrv::digitalWrite(25, HIGH);WiFiDrv::digitalWrite(26, LOW);  WiFiDrv::digitalWrite(27, LOW); }
void LedBlue(){  WiFiDrv::digitalWrite(25, LOW); WiFiDrv::digitalWrite(26, LOW);  WiFiDrv::digitalWrite(27, HIGH); }



void ensureWiFi(){
  if (WiFi.status() == WL_CONNECTED) return;
  
  // Check if the WiFi module is present
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // Don't continue if module is not present
    while (true) delay(1000); 
  }
  
  LedBlue();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  // Attempt to connect to WiFi network
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED){
    Serial.println("Wi-Fi is disconnected. Re-running setup_wifi...");
    status = WiFi.begin(ssid, pass);
    delay(500);
  }
  
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  LedGreen();
}

void ensureMQTT(){
  if (mqttClient.connected()) return;
  if (millis() - lastConnectionAttempt < RECONNECT_INTERVAL_MS) return;
  lastConnectionAttempt = millis();

  LedBlue();
  Serial.print("MQTT connecting… ");
  if (mqttClient.connect(mqtt_client_id)){
    Serial.println("OK");
    bool ok = true;
    ok &= mqttClient.subscribe(user_update_topic);
    ok &= mqttClient.subscribe(brightness_update_topic);
    ok &= mqttClient.subscribe(mqtt_data_topic);
    if (ok) LedGreen(); else LedRed();
  } else {
    Serial.print("fail rc="); Serial.println(mqttClient.state());
    LedRed();
  }
}

void publishCmdJSON(const char* palette, uint16_t seconds, const char* faceName){
  // Small, dependency-free JSON
  char buf[200];
  snprintf(buf, sizeof(buf),
    "{\"device\":\"%s\",\"cmd\":\"timer\",\"palette\":\"%s\",\"seconds\":%u,\"face\":\"%s\"}",
    mqtt_client_id, palette, seconds, faceName ? faceName : "");
  mqttClient.publish(mqtt_cmd_topic, buf);
  Serial.print("MQTT cmd → "); Serial.print(mqtt_cmd_topic); Serial.print("  ");
  Serial.println(buf);
}

void showAll(uint8_t r,uint8_t g,uint8_t b){
  for (int i=0;i<NEOPIXEL_COUNT;i++) pixels.setPixelColor(i, r,g,b);
  pixels.show();
}

/********* MQTT CALLBACK *********/
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // Convert the payload to a C-style string for easier manipulation
  // memcpy and payload_str[length] = '\0' lines are a standard way to safely convert 
  // the received byte array payload into a null-terminated string (char*), 
  // which is required for functions like atoi.
  char payload_str[length + 1];
  memcpy(payload_str, payload, length);
  payload_str[length] = '\0'; 
  
  // Print the received message for debugging
  //Serial.print("Message received on topic: ");
  //Serial.println(topic);
  //Serial.print("Message: ");
  //Serial.println(payload_str);

  // 1. Check if the topic is for updating the LUMINAIRE_USER variable
  // strcmp returns 0 if the two strings are identical
  if (strcmp(topic, user_update_topic) == 0) {
    int new_user_id = atoi(payload_str);
    
    // Only update if the user ID has actually changed
    if (new_user_id != LUMINAIRE_USER) {
      
      // Unsubscribe from the OLD topic
      Serial.print("Unsubscribing from old topic: ");
      Serial.println(mqtt_data_topic);
      mqttClient.unsubscribe(mqtt_data_topic);

      // Update the global LUMINAIRE_USER variable
      LUMINAIRE_USER = new_user_id;

      // Generate the NEW topic string
      snprintf(mqtt_data_topic, sizeof(mqtt_data_topic), "%s/%d", mqtt_base_topic, LUMINAIRE_USER);

      // Subscribe to the NEW topic
      if (mqttClient.subscribe(mqtt_data_topic)) {
        Serial.print("Subscribed to NEW topic: ");
        Serial.println(mqtt_data_topic);
      } else {
        Serial.println("Failed to subscribe to NEW topic!");
      }
      
      Serial.print("LUMINAIRE_USER updated to: ");
      Serial.println(LUMINAIRE_USER);

      pixels.clear();
      pixels.show();

    } else {
      Serial.println("LUMINAIRE_USER is already set to this ID. No change.");
    } 
    
  } else if (strcmp(topic, brightness_update_topic) == 0){
    // Convert the string payload to an integer
    int new_brightness_id = atoi(payload_str);
    
    // Update the global LUMINAIRE_USER variable
    LUMINAIRE_BRIGHTNESS = new_brightness_id;
    
    Serial.print("LUMINAIRE_BRIGHTNESS updated to: ");
    Serial.println(LUMINAIRE_BRIGHTNESS);  
    pixels.setBrightness(LUMINAIRE_BRIGHTNESS); // Set a default brightness (0-255) to avoid blinding light
  
  }
  // 2. Check if the topic ends with the current LUMINAIRE_USER ID
  else {
    // Find the last '/' in the topic string to isolate the number
    char* last_slash = strrchr(topic, '/');
    if (last_slash != NULL) {
      // Move the pointer past the slash
      char* number_str = last_slash + 1;
      int topic_user_id = atoi(number_str);
      
      // Compare the extracted number with our current LUMINAIRE_USER
      if (topic_user_id == LUMINAIRE_USER) {
        //Serial.print("This message is for my luminaire (ID: ");
        //Serial.print(LUMINAIRE_USER);
        //Serial.println("). Processing payload...");
        
        // Add your specific code to process the payload here.
        // For example, control a light or other component.
        // For now, we'll just print it.
        //Serial.print("Payload to process: ");
        //Serial.println(payload_str);


        // Check if the payload length matches the expected length for all LEDs
        if (length == NEOPIXEL_DATA_LENGTH) {
          // Iterate through the payload, 3 bytes at a time for each LED
          for (int i = 0; i < NEOPIXEL_COUNT; i++) {
            // Extract RGB values. Payload format: R1, G1, B1, R2, G2, B2, ...
            // NeoPixel library expects GRB order for WS2812B, but we set it as RGB
            // and the library handles the conversion if NEO_GRB is used.
            // So, payload[index] is Red, payload[index+1] is Green, payload[index+2] is Blue.
            byte r = payload[i * 3];     // Red component
            byte g = payload[i * 3 + 1]; // Green component
            byte b = payload[i * 3 + 2]; // Blue component

            // Set the color for the current pixel
            pixels.setPixelColor(i, r, g, b);
          }
          // Update the NeoPixels to show the new colors
          pixels.show();
          //Serial.println("NeoPixels updated!");
        } else {
          //Serial.print("Warning: Received payload length (");
          //Serial.print(length);
          //Serial.print(") does not match expected length (");
          //Serial.print(NEOPIXEL_DATA_LENGTH);
          //Serial.println(") for 48 RGB LEDs. Ignoring update.");
        }




      } else {
       //Serial.print("This message is for a different luminaire (ID: ");
        //Serial.print(topic_user_id);
        //Serial.println("). Ignoring.");
      }
    }
  }
}

/********* IMU SETUP *********/
void imuSetup(){
  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();
  if (!mpu.testConnection()){
    Serial.println("MPU connect FAIL");
    while(1);
  }
  int st = mpu.dmpInitialize();
  if (st != 0){
    Serial.print("DMP init fail: "); Serial.println(st);
    while(1);
  }
  //mpu.dmpSetFIFORate(50); // if available in your header
  mpu.setDMPEnabled(true);

  // Polling mode (INT optional)
  mpu.setInterruptMode(false);
  mpu.setInterruptDrive(false);
  mpu.setInterruptLatch(true);
  mpu.setInterruptLatchClear(true);
  mpu.setIntEnabled(0x02);

  packetSize = mpu.dmpGetFIFOPacketSize();
  dmpReady = true;

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

  float t = (float)el / (float)dur;
  if (t<0) t=0; if (t>1) t=1;

  uint8_t r=0,g=0,b=0;
  if (programIsSquare){ r=255; g=(uint8_t)(255*t); b=0; }         // red → yellow
  else                 { r=0;   g=(uint8_t)(255*t); b=(uint8_t)(255*(1-t)); } // blue → green

  float phase = (now % BLINK_PERIOD_MS) / (float)BLINK_PERIOD_MS;
  float env   = (phase < 0.5f) ? (phase*2.0f) : (2.0f - phase*2.0f);
  float scale = 0.25f + 0.75f * env;

  showAll((uint8_t)(r*scale),(uint8_t)(g*scale),(uint8_t)(b*scale));
}

/********* ARDUINO SETUP *********/
void setup(){
  // Initialize serial communication for debugging
  Serial.begin(115200);
  while(!Serial);// Wait for serial port to connect (useful for debugging)

  // RGB LED's
  WiFiDrv::pinMode(25, OUTPUT); // G
  WiFiDrv::pinMode(26, OUTPUT); // R
  WiFiDrv::pinMode(27, OUTPUT); // B

  LedRed();

  Serial.println("Starting MKR1010 NeoPixel MQTT Controller...");

  // Initialize NeoPixels
  pixels.begin(); // Initialize the NeoPixel library
  pixels.show();  // Turn all pixels off initially
  pixels.setBrightness(LUMINAIRE_BRIGHTNESS); // Set a default brightness (0-255) to avoid blinding light


  // Set the MQTT server and callback function
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqtt_callback);

  // Generate the initial data subscription topic ---
  // Format: "student/CASA0014/luminaire/0"
  snprintf(mqtt_data_topic, sizeof(mqtt_data_topic), "%s/%d", mqtt_base_topic, LUMINAIRE_USER);
  Serial.print("Initial data topic set to: ");
  Serial.println(mqtt_data_topic);

  // Connect to Wi-Fi
  
  ensureWiFi();
  ensureMQTT();

  // IMU
  imuSetup();

  LedGreen();
}

/********* ARDUINO LOOP *********/
void loop(){
  // Keep connections alive
  ensureWiFi();
  ensureMQTT();
  mqttClient.loop();

  // IMU polling
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

    // Up vector = -gravity
    float ux = -gravity.x;
    float uy = -gravity.y;
    float uz = -gravity.z;

    // Find best face
    int bestIdx = -1; float bestDot = -2.0f;
    for (int i=0;i<14;i++){
      float d = dot3(ux,uy,uz, ALL[i].x,ALL[i].y,ALL[i].z);
      if (d > bestDot){ bestDot = d; bestIdx = i; }
    }
    bool valid = false;
    if (bestIdx >= 0){
      valid = ALL[bestIdx].isSquare ? (bestDot >= COS_THR_SQUARE)
                                    : (bestDot >= COS_THR_TRIANGLE);
    }
    int detectedFace = valid ? bestIdx : -1;

    // Stability + trigger
    static unsigned long faceChangeLocal = millis();
    unsigned long now = millis();

    if (detectedFace != lastFace){
      lastFace = detectedFace;
      faceChangeLocal = now;
      faceLocked = false;
      if (programRunning) programRunning = false; 
    } else {
      if (!faceLocked && detectedFace >= 0 && (now - faceChangeLocal >= STABLE_REQUIRED_MS)){
        faceLocked = true;
        currentFace = detectedFace;

        programIsSquare = ALL[currentFace].isSquare;
        programRunning  = true;
        programStart    = now;

        if (programIsSquare){
          publishCmdJSON("red_yellow", DURATION_SQUARE_MS/1000, ALL[currentFace].name);
          Serial.println("Locked SQUARE → 30s red_yellow");
        } else {
          publishCmdJSON("blue_green", DURATION_TRIANGLE_MS/1000, ALL[currentFace].name);
          Serial.println("Locked TRIANGLE → 10s blue_green");
        }
      }
    }
  }

  runProgramPreview(); 
}
