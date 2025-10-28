// It’s a tiny Serial + timing  test for MKR1010.
// What it verifies:
// USB Serial link works: Serial.begin(115200) opens the port; 
// while (!Serial) {} waits until the Serial Monitor is opened.
// Board is running and not crashing: you’ll see a steady stream of prints.
// Timing with millis() is sane: it prints once per second using a non-blocking timer.


void setup() {
  Serial.begin(115200);
  while (!Serial) {}   // wait for monitor
  Serial.println("Serial OK. Hello from MKR1010!");
}
void loop() {
  static unsigned long t=0;
  if (millis()-t>1000) { t=millis(); Serial.println(millis()); }
}
