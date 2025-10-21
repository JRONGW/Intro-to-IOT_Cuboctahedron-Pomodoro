void setup() {
  Serial.begin(115200);
  while (!Serial) {}   // wait for monitor
  Serial.println("Serial OK. Hello from MKR1010!");
}
void loop() {
  static unsigned long t=0;
  if (millis()-t>1000) { t=millis(); Serial.println(millis()); }
}
