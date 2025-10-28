# Cuboctahedron Pomodoro

<img width="3508" height="2480" alt="A4 cuboctahedron" src="https://github.com/user-attachments/assets/4d2e69e6-a9f6-42c0-9407-53acfa2cd62a" />

- The core functionality of the Cuboctahedron Pomodoro is the use it as a timer for work and rest. Cuboctahedron is a symmetrical geometry representing the bi-mode of this pomodoro with its 6 identical squares and 8 identical triangles. The larger square surface down sets a 25min clock for work. The smaller triangle surface down sets a 5min clock for rest. In the testing, I use the 20s as a substitution for 25min clock and 10s as a substitution for 5min clock. 

### Merits
- It connects physical interaction, edge computing, event-driven networks and visualization into a closed loop.
- Edge-side filtering + hysteresis + dwell time can eliminate "false triggering" at the device end and reduce the noise of uplink data (the value of edge computing).

###  Robustness
####  Unit-vector normalization + zero-guard: 
- I compute n2 and bail if it’s ≈0, then normalize → prevents NaNs and ensures stable dot products. If I don’t normalize the accelerometer vector, the direction test gets contaminated by magnitude, which breaks your thresholds, hysteresis, and dwell logic. Because the thresholds are set assuming that the n2 is normalized
####  LPF + re-normalize: 
- exponential smoothing (LPF_ALPHA) damps jitter; re-normalizing keeps math stable.
####  Hysteresis (ENTER_TH/EXIT_TH): 
- avoids flicker when the score hovers near the threshold.
####  10-second dwell confirmation (DWELL_MS): 
- suppresses false triggers from bumps.
####  Re-arm dip (REARM_DROP_MS)
- After confirming a face, don’t consider any new face until the current face’s score has dipped below a lower threshold (EXIT_TH) for at least REARM_DROP_MS milliseconds.
####  Last-confirmed suppression: 
- ignores the same face immediately after confirming it to prevent repeats.


### Demerits
- The function of the pomodoro needs to be explained to the user. 

### Design
- The design follows the design principle that form follows functions in architecture. This geometry works as an abstract switch with correspondence from shape to time. 
- The larger surface with 4 edges means a longer time of working mode. The smaller surface with 3 edges means a shorter time of working mode.

### What I learned
- While making my prototype, I learned to design an algorithm to distinguish the faces from the sensor data. I tried ypr angles and acceleration. The acceleration worked. 

#### Algorithm for face detection using acceleration from IMU(MPU 6050)
- Maximizing -cos(θ) finds the θ when θ ≈ π (180°): the vectors are opposite.
- So the code is effectively finding the face whose outward normal n is most opposite to gravity g—equivalently, the face whose inward normal -n is most aligned with g.
- It’s picking the one closest to -N6[i] (i.e., the face that’s pointing down). The same logic is then repeated for the 8 triangle-face normals in N8.

 <img width="568" height="282" alt="image" src="https://github.com/user-attachments/assets/9597d6ca-9d3c-4f11-8b39-0d76055093ae" />


# Circuit in Progress


<img width="320" height="239" alt="image" src="https://github.com/user-attachments/assets/b321b5eb-fd0a-4e08-b055-28223f4aff71" />




<img width="362" height="365" alt="image" src="https://github.com/user-attachments/assets/a39b54cc-271a-4304-b3a2-c8eaf5cdc51b" />



# Video of Progress

<img width="122" height="190" alt="image" src="https://github.com/user-attachments/assets/fb59ec73-2fa9-4f60-a899-0affc355ef8c" />

Face detection tested successfully:
https://github.com/user-attachments/assets/b0937933-bc55-468b-9377-7d1f1f2e085a

<img width="114" height="110" alt="screenshot" src="https://github.com/user-attachments/assets/b04b7198-1a45-4b83-b5b4-2d38504dfdb8" />

Cuboctahedron Pomodoro to Vespera
https://github.com/user-attachments/assets/1aa174da-ec46-4a6b-9827-48664f9fa585

<img width="121" height="148" alt="image" src="https://github.com/user-attachments/assets/e6b62a3c-b3cb-456e-9848-38f170152dd5" />

Effects updated
https://github.com/user-attachments/assets/3790d0ce-7d41-47c6-a9a7-6038a02cd146



# Code for Face Detection, MQTT Connection, and Light Control - lightconnect.ino
## Hardware/Overall Structure
- Device Layer: MKR WiFi 1010 (WiFiNINA module + on-board RGB), MPU-6050 (accelerometer).
- Connection Layer: MQTT (tcp 1884).
- Application Layer: 72-pixel matrix web page + Vespera device (216-byte RGB frame), achieving synchronous visualization of "digital twin".

## 72 Pixels and Frame Buffer
- One-dimensional buffer frameBuf[216], logically mapped as 6 rows × 12 columns.
- Index function: idxFromRowCol(r,c) = c*6 + r (column-major order, 6 pixels per column).
- After each rendering, it is pushed to the web page/device via mqttClient.publish(mqtt_data_topic, frameBuf, 216).

## Wi-Fi and MQTT connection
- The `ensureWiFi()` function continuously attempts to connect; once successful, it lights up the on-board green LED (PWM pins 25/26/27 of NINA). 
- The `ensureMQTT()` function tries to connect every 5 seconds; upon success, it subscribes to three types of topics.

## IMU orientation determination ("which side is down")
### Sampling and Normalization
- Read the raw acceleration (ax, ay, az), and normalize it to (gx, gy, gz) ≈ the gravity direction vector at 16384 LSB/g.
- Low-pass filtering: Apply a first-order IIR smoothing with LPF_ALPHA = 0.20 and renormalize to suppress jitter.
### Face Vector Library
- 6 square face normals: ±X, ±Y, ±Z.
- 8 triangular face normals: Unit vectors in the cube's body diagonal directions (±1, ±1, ±1)/√3.
### Matching Criterion: Seek the face with the maximum -dot(n, g).
- If a face normal n is approximately opposite to the current gravity g, a larger -dot(n, g) indicates that the face is more "correctly" facing down.
### Hysteresis and Stability
- Enter threshold ENTER_TH = 0.80, exit threshold EXIT_TH = 0.72 (to prevent oscillation).
- Prevent "re-triggering on the same face": After confirming a face, the next confirmation must be on a different face.
### Dwell Confirmation (to avoid false triggers)
- A confirmation is only made after remaining on the same candidate face for 10 seconds (DWELL_MS).
- Upon confirmation, start the corresponding visual program (square → work mode, triangle → rest mode).

# Testing Codes
### Test I²C - I2Cscanner0.ino
- That’s an I²C bus scanner. It tests whether anything is alive on the I²C lines and what 7-bit addresses they respond to.
- What it verifies:
-   Wiring & power: SDA/SCL connected correctly, device powered, grounds tied together.
-   Device presence & address: prints any responding address (e.g., MPU-6050 at 0x68; 0x69 if AD0=HIGH).
-   Bus health: if nothing shows, there may be shorts, missing pull-ups, wrong voltage, or a stuck device.


### Test MPU-6050 and I²C wiring - minimaltestMakeINTeasytocatch.ino
- This sketch is a hardware/debug probe for  MPU-6050 DATA_RDY interrupt and I²C wiring. It checks three things:
- What it sets up
-   I²C comms to the MPU-6050 at address 0x68 (change to 0x69 if AD0=HIGH).
-   INT pin behavior on the MPU:
-   Enables DATA_RDY interrupt (INT_ENABLE = 0x01).
-   Tries to set: active-HIGH, push-pull, latched until cleared, clear-on-INT_STATUS read (via the library calls).
-   MCU interrupt line: connects MPU’s INT pin → MKR1010 D2, and attaches attachInterrupt(..., RISING) to catch edges.
- What it prints (and why)
-   On setup() it reads key registers directly (even if the lib lacks getByte):
-   WHO_AM_I → should be 0x68 (or 0x69 depending on variant).
-   PWR_MGMT_1 → expect 0x00 if not sleeping.
-   INT_ENABLE → expect 0x01 (DATA_RDY enabled).
-   INT_PIN_CFG → shows pin config bits (latch, polarity, etc.).

### Test MPU-6050 angle - mpu6050_mkr1010_new.ino
-   This tests printing the ypr angles from MPU-6050

### Test the face detection with UNO when MKR1010 is broken - unotest.ino
-   This tests which face is down using MPU-6050 and Arduino UNO

### Test the face detection with UNO when MKR1010 is broken - quickserialsanitytest.ino
-   It’s a tiny Serial + timing test for MKR1010.
-   What it verifies:
-   USB Serial link works: Serial.begin(115200) opens the port; 
-   while (!Serial) {} waits until the Serial Monitor is opened.
-   Board is running and not crashing: you’ll see a steady stream of prints.
-   Timing with millis() is sane: it prints once per second using a non-blocking timer.

### Test the MPU-6050 exporting - MPU6050exporttest.ino
-   I²C wiring & power
-     Wire.begin(); Wire.setClock(400000); mpu.initialize(); mpu.testConnection()//Confirms the chip responds on I²C (0x68/0x69) at 400 kHz.
-   DMP firmware load & configuration

-     mpu.dmpInitialize(); mpu.setDMPEnabled(true); packetSize = mpu.dmpGetFIFOPacketSize();//Ensures the on-chip DMP successfully boots and outputs fixed-size packets (typically 42 bytes).
-   FIFO health
-     fifoCount = mpu.getFIFOCount(); ... if (fifoCount == 1024) resetFIFO();//Checks for overflows
-   Quaternion → gravity → YPR math
-     dmpGetQuaternion(&q, fifoBuffer);
-     dmpGetGravity(&gravity, &q);
-     dmpGetYawPitchRoll(ypr, &q, &gravity);//Confirms the DMP orientation solution is coherent and  derives yaw, pitch, roll.
-   Streaming/throughput check: prints YPR at ~20 Hz (if (now - last >= 50)) to make sure Serial USB isn’t the bottleneck.

## Effect state machine (EFFECT_*)
-   EFFECT_SQUARE (20 s): "Row scan + row color breathing"
-   EFFECT_TRIANGLE (10 s): "Veered single-pixel column drift" (added the veeredSinglePerColGradient effect)
-   EFFECT_YELLOW: "5-pixel gradient chase (black ↔ yellow ↔ black)"

## Effect functions
### Row "Blink Through" Gradient - rowBlinkThrough()
-   Core idea: Instead of moving the "gradient stripes", the active row/column itself "breathes" between colors A and B over time.
-   Background: Static gradient from white to the target color (row: #FBE9D7; column: #E0F4FF) to enhance the layering effect.
-   Renderer:
-   rowBlinkThrough(r, a, b, bgTarget, t): First draw the column-wise gradient for the entire background, then overlay the r-th row with lerp(a, b, t).

### diagonal drift single pixel + vertical halo - veeredSinglePerColGradient()
-   veeredSinglePerColGradient(startCol, windowCols, baseRow, veer, a, b, radius)
-   Each column lights up one pixel and scrolls with time starting from startCol.
-   Every DRIFT_EVERY steps, baseRow moves down, forming a diagonal "snake-like" drift.
-   The upper and lower parts of the same column create a halo that gradually fades to black with a radius, enhancing the three-dimensional effect.
-   Visually, it is more dynamic than fixed "column strips".

### The "Chaser" and "Flip Every Lap" - chaserWindow()
-   chaserWindow(start, 5, a, b): A 5-pixel window advances in a 72-pixel loop.
-   chaseFlip flips a ↔ b every lap, creating an alternating pattern of light blue ↔ black ("color change every lap").
-   synchronized the on-board RGB to the tail color cB, creating a mini debug/heartbeat indicator, which is very practical.

# Design Files
-   Cuboctahedron Components folder - contains stl files of componets
-   Cuboctahedron Pomodoro.3dm -  cuboctahedron pomodoro prototype model file

