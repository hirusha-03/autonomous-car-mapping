/*
  Autonomous Exploration Robot — ESP32 Firmware
  Hardware:
    HC-SR04 front (physically mounted at rear of chassis): TRIG=GPIO18, ECHO=GPIO19
    HC-SR04 left  (~30deg off front): TRIG=GPIO4,  ECHO=GPIO5
    HC-SR04 right (~30deg off front): TRIG=GPIO15, ECHO=GPIO13
    L298N: IN1=GPIO25, IN2=GPIO26, IN3=GPIO27, IN4=GPIO14
    ENA=GPIO32, ENB=GPIO33 (PWM speed control, user-adjustable per side via
    /speed — see motorSpeedLeftPct/motorSpeedRightPct below). Turns always
    run at full PWM regardless of that setting; see turnLeft/turnRight.
    MPU6050 (I2C): SDA=GPIO21, SCL=GPIO22, VCC=3.3V, GND=GND

  Networking: robot traffic (sensor stream, motor commands, stop, obstacle
  reports) rides one persistent WebSocket (/ws) instead of per-action HTTP
  requests — a fresh HTTP connect/teardown on every loop iteration (plus a
  /stop_flag poll every 150ms mid-motion) was the main source of perceived
  lag. Calibration reporting (/calibrate/report) stays on plain HTTP since
  it's post-motion, not latency-sensitive, and only runs during manual
  calibration sessions, not normal driving.
  Requires the "WebSockets" library by Markus Sattler (arduinoWebSockets).
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Wire.h>

// ── CONFIG ───────────────────────────────────────────────────────────────────
const char* WIFI_SSID     = "Hirusha’s iPhone";
const char* WIFI_PASSWORD = "HnH@141414";
const char* SERVER_HOST   = "172.20.10.2";
const uint16_t SERVER_PORT = 8000;
const char* WS_PATH        = "/ws";
// Still used for the calibration HTTP endpoints only (see bottom of file).
const char* SERVER_URL    = "http://172.20.10.2:8000";

// Calibration — tune these on the real robot
// Recalibrated via the /calibrate/turn bisection tool: 163ms was overshooting
// to ~180°, real 90° measured at ~80.5ms. delay() only takes whole ms, so
// this is rounded to the nearest integer (negligible vs. run-to-run drift).
// Also used by turnByAngle() as the fallback timing when the MPU6050 isn't
// available, and as the basis for its gyro-stall time cap.
const int TURN_90_MS  = 81;
// Cell size changed to 30.48cm (1ft) — half of the previous 60.96cm (2ft)
// cell, so this is scaled back down proportionally; still pending a real
// on-robot recalibration (distance-per-ms isn't perfectly linear in
// practice — motor stall torque, battery sag, etc).
const int FORWARD_MS  = 800;
// Matches sensorsweep.py/app.js's cell size assumption — used to compute the
// "commanded_cm" calibration field (expected distance implied by FORWARD_MS
// timing) reported alongside each forward/reverse move.
const float CELL_SIZE_CM = 30.48f;

// Motion delay() is split into chunks this long, checking for a pushed stop
// frame (webSocket.loop() dispatches it via onWsEvent) between each chunk —
// so a manual stop or an obstacle can abort a drive/turn already in progress
// instead of waiting out the full duration_ms blind.
const int STOP_POLL_CHUNK_MS = 150;

// Forward drive self-aborts if ANY of the three sensors reads closer than
// this mid-motion. Checking all three (not just front) closes the gap where
// an obstacle approached at the ~30deg angle the side sensors cover was
// never checked during a drive, only reported after the fact. Independent
// of the server's occupancy-grid obstacle ratio (sensorsweep.py) — this is
// purely a physical collision-avoidance cutoff, intentionally tighter than
// one grid cell (30cm) so it trips before contact.
const float SAFETY_STOP_CM = 15.0;

// ── PIN DEFINITIONS ──────────────────────────────────────────────────────────
#define TRIG_PIN 18
#define ECHO_PIN 19
#define TRIG_PIN_L 4
#define ECHO_PIN_L 5
#define TRIG_PIN_R 15
#define ECHO_PIN_R 13
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14
#define ENA_PIN 32   // PWM speed for motor A / left side (IN1/IN2) — jumper removed
#define ENB_PIN 33   // PWM speed for motor B / right side (IN3/IN4) — jumper removed
// If left/right feel swapped on the real chassis, swap ENA_PIN/ENB_PIN here
// rather than relabeling anything server-side.
#define MPU_SDA 21
#define MPU_SCL 22
#define MPU_ADDR 0x68

bool mpuOk = false;

WebSocketsClient webSocket;

// Set by onWsEvent() when a {"type":"command"} frame arrives; consumed once
// per loop() iteration. Only one command is ever in flight — the server only
// pushes the next one after we send {"type":"ready"} for the previous one.
volatile bool newCommandAvailable = false;
String pendingCmd = "none";
int pendingDurationMs = 500;

// Edge-triggered: set by onWsEvent() on a {"type":"stop"} frame, consumed by
// runMotion's chunk loop. No HTTP poll needed — the server pushes this the
// instant a manual stop or nav abort happens.
volatile bool stopRequested = false;

// Requires ESP32 Arduino core >=3.0 (analogWrite is PWM-backed there; on
// older cores use ledcAttach/ledcWrite instead).
// Independent per-side rather than one speed + a fixed trim: measured drift
// direction wasn't consistent across test runs, so a single fixed-direction
// correction would assume a bias that doesn't always hold — these are
// user-adjustable instead.
int motorSpeedLeftPct = 70;
int motorSpeedRightPct = 70;

// ── SENSOR ───────────────────────────────────────────────────────────────────
// Fired one at a time (not simultaneously) — HC-SR04s cross-talk if triggered
// together, each echo can pick up another sensor's ultrasonic pulse.
float readDistanceOn(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0) return 400.0;
  return duration * 0.0343f / 2.0f;
}

float readDistance() {
  return readDistanceOn(TRIG_PIN, ECHO_PIN);
}

// ── IMU ──────────────────────────────────────────────────────────────────────
// Raw register access rather than the Adafruit_MPU6050 library — that library
// refuses to init on some clone GY-521 boards because it hard-checks the
// WHO_AM_I register, even though the chip communicates fine over I2C. Talking
// to the registers directly (same approach as the working sanity-check sketch)
// sidesteps that check entirely.
// gyroZ (rad/s) is the useful bit for this robot — yaw rate around the
// vertical axis, i.e. how fast it's turning. Accel is reported too in case
// it's useful later (tilt/collision detection) but nothing currently uses it.
// Scale factors assume default full-scale ranges (no config write below):
// accel +/-2g -> 16384 LSB/g, gyro +/-250 deg/s -> 131 LSB/(deg/s).
// GYRO_GAIN/GYRO_BIAS_DPS correct systematic gyro error (constant offset while
// still, and a scale-factor error vs. true rotation) — fit from real protractor
// measurements via calibrate_fit.py against logged /calibrate/report data.
// Defaults are no-ops until that fit is done once on the real robot.
const float GYRO_GAIN = 1.0f;
const float GYRO_BIAS_DPS = 0.0f;

void readImu(float& accelX, float& accelY, float& accelZ, float& gyroZ) {
  if (!mpuOk) { accelX = accelY = accelZ = gyroZ = 0; return; }

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) {
    accelX = accelY = accelZ = gyroZ = 0;
    return;
  }
  Wire.requestFrom(MPU_ADDR, 14, true);
  if (Wire.available() < 14) { accelX = accelY = accelZ = gyroZ = 0; return; }

  int16_t rawAX = Wire.read() << 8 | Wire.read();
  int16_t rawAY = Wire.read() << 8 | Wire.read();
  int16_t rawAZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();          // TEMP_OUT, unused
  int16_t rawGX = Wire.read() << 8 | Wire.read();
  int16_t rawGY = Wire.read() << 8 | Wire.read();
  int16_t rawGZ = Wire.read() << 8 | Wire.read();
  (void)rawGX; (void)rawGY;          // only yaw rate (Z) is currently used

  accelX = rawAX / 16384.0f * 9.80665f;
  accelY = rawAY / 16384.0f * 9.80665f;
  accelZ = rawAZ / 16384.0f * 9.80665f;
  float gzDps = (rawGZ / 131.0f - GYRO_BIAS_DPS) * GYRO_GAIN;
  gyroZ = gzDps * (PI / 180.0f);
}

// ── MOTORS ───────────────────────────────────────────────────────────────────
void applySpeed() {
  analogWrite(ENA_PIN, motorSpeedLeftPct * 255 / 100);
  analogWrite(ENB_PIN, motorSpeedRightPct * 255 / 100);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
}

void setMotorPins(int in1, int in2, int in3, int in4) {
  digitalWrite(IN1, in1); digitalWrite(IN2, in2);
  digitalWrite(IN3, in3); digitalWrite(IN4, in4);
  applySpeed();
}

// Runs the motion for `ms` in STOP_POLL_CHUNK_MS steps, checking for a pushed
// stop frame between each (webSocket.loop() is what actually dispatches an
// incoming frame to onWsEvent — must be called every chunk so a stop lands
// promptly instead of waiting for the next full loop() iteration). If
// checkObstacle is true (forward drive only), also checks all three sensors
// between chunks and self-aborts before contact — this is what closes the
// "blind for 800ms per cell" gap: previously the sensor was only read once,
// before the drive started, so anything that appeared mid-drive went
// undetected until impact. hitSide (if non-null) reports which sensor
// tripped: 'F' (front), 'L', 'R', or 0 if none/interrupted by manual stop
// instead — used for /obstacle_stop's side field. completed (if non-null)
// reports whether the motion ran to completion with no interruption at all
// (obstacle OR manual stop) — used to decide whether a calibration sample is
// trustworthy. accelDistanceCm (if non-null), when checkObstacle's caller
// also wants distance tracking, accumulates a double-integrated forward-axis
// distance estimate once per chunk — coarse (chunk-rate sampling, ~150ms) and
// prone to drift like any double-integrated accelerometer estimate, but good
// enough to log as a comparison point against a real tape-measure reading.
bool runMotion(int ms, bool checkObstacle = false, char* hitSide = nullptr,
                bool* completed = nullptr, float* accelDistanceCm = nullptr) {
  bool interrupted = false;
  char hit = 0;
  int remaining = ms;
  float velocity = 0;   // cm/s, integrated from forward-axis accel
  float distance = 0;   // cm
  unsigned long lastMs = millis();

  while (remaining > 0) {
    webSocket.loop();  // dispatch any pushed stop frame to onWsEvent

    // Check BEFORE each chunk (including the last) rather than after — the
    // old order skipped the check ahead of the final chunk entirely (it hit
    // "remaining <= 0, break" first), so the robot drove its last ~150ms
    // blind with no live check right as it got closest to an obstacle.
    if (checkObstacle) {
      // Staggered like loop()'s own sweep — HC-SR04s cross-talk if fired
      // together.
      if (readDistanceOn(TRIG_PIN, ECHO_PIN) < SAFETY_STOP_CM) {
        hit = 'F';
      } else {
        delay(20);
        if (readDistanceOn(TRIG_PIN_L, ECHO_PIN_L) < SAFETY_STOP_CM) {
          hit = 'L';
        } else {
          delay(20);
          if (readDistanceOn(TRIG_PIN_R, ECHO_PIN_R) < SAFETY_STOP_CM) hit = 'R';
        }
      }
      if (hit) {
        Serial.printf("[SAFETY] Obstacle within stop distance (%c) — aborting motion early\n", hit);
        interrupted = true;
        break;
      }
    }
    if (stopRequested) {
      stopRequested = false;
      Serial.println("[STOP] Manual stop — aborting motion early");
      interrupted = true;
      break;
    }

    if (accelDistanceCm) {
      float ax, ay, az, gz;
      readImu(ax, ay, az, gz);
      unsigned long now = millis();
      float dt = (now - lastMs) / 1000.0f;
      lastMs = now;
      // Assumes MPU6050 X axis is the robot's forward/back axis — same
      // mounting assumption as noted where FORWARD ACCEL is used elsewhere;
      // swap to ay here if the board turns out mounted rotated 90 degrees.
      velocity += ax * 100.0f * dt;   // m/s^2 -> cm/s^2, integrate to cm/s
      distance += fabs(velocity) * dt;
    }

    int chunk = remaining < STOP_POLL_CHUNK_MS ? remaining : STOP_POLL_CHUNK_MS;
    delay(chunk);
    remaining -= chunk;
  }

  stopMotors();
  if (hitSide) *hitSide = hit;
  if (completed) *completed = !interrupted;
  if (accelDistanceCm) *accelDistanceCm = distance;
  return !interrupted;
}

void driveForward(int ms, char& hitSide, bool& completed, float& accelDistanceCm) {
  // Motor polarity swapped: sensorsweep.py assumes the HC-SR04 faces the
  // direction of travel (it marks cells ahead of robot.direction as sensed).
  // The old pin pattern drove the robot away from the sensor end, so the
  // sensor was actually reading behind the robot. Swapping polarity here
  // makes "forward" drive toward the sensor end, matching that assumption.
  setMotorPins(LOW, HIGH, LOW, HIGH);
  runMotion(ms, /*checkObstacle=*/true, &hitSide, &completed, &accelDistanceCm);
}

void driveReverse(int ms, bool& completed, float& accelDistanceCm) {
  // Opposite polarity of driveForward — drives away from the sensor end.
  // No rear sensor, so no live obstacle check here.
  setMotorPins(HIGH, LOW, HIGH, LOW);
  runMotion(ms, /*checkObstacle=*/false, nullptr, &completed, &accelDistanceCm);
}

// Turns run at a fixed, full-power PWM regardless of motorSpeedLeftPct/
// motorSpeedRightPct — TURN_90_MS was calibrated at full power (see above),
// so scaling turn speed with the user's forward/reverse speed slider would
// throw off the ms-per-degree calibration (this was the cause of turns
// falling well short of 90°).
void turnLeft(int ms) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA_PIN, 255);
  analogWrite(ENB_PIN, 255);
  runMotion(ms);
}

void turnRight(int ms) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA_PIN, 255);
  analogWrite(ENB_PIN, 255);
  runMotion(ms);
}

// Gyro-corrected turn: spins at full PWM (same direction pins as turnLeft/
// turnRight) until the integrated gyro_z angle reaches targetDeg, instead of
// trusting a fixed ms duration — closes the drift gap noted above (motor
// stall torque, battery sag make ms-per-degree inconsistent run to run).
// Falls back to the old fixed-ms behavior if the MPU6050 isn't available, so
// the robot still turns (just less precisely) without it.
// Returns the gyro-integrated angle actually achieved (0 if mpuOk was false —
// there's no gyro estimate to report in that case), so the caller can log it
// against a real protractor measurement via /calibrate/report.
float turnByAngle(bool isLeft, float targetDeg, int fallbackMs) {
  if (isLeft) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  }
  analogWrite(ENA_PIN, 255);
  analogWrite(ENB_PIN, 255);

  if (!mpuOk) {
    delay(fallbackMs);
    stopMotors();
    return 0;
  }

  const unsigned long MIN_MS = 20;     // ignore angle check briefly at start — avoids an
                                        // early I2C noise spike causing an instant false stop
  const unsigned long capMs = (unsigned long)fallbackMs * 3;  // ceiling if gyro stalls/reads zero

  float accumulatedDeg = 0;
  unsigned long startMs = millis();
  unsigned long lastMs = startMs;

  while (true) {
    unsigned long now = millis();
    float dt = (now - lastMs) / 1000.0f;
    lastMs = now;

    float ax, ay, az, gz;
    readImu(ax, ay, az, gz);
    accumulatedDeg += gz * (180.0f / PI) * dt;

    unsigned long elapsed = now - startMs;
    if (elapsed >= MIN_MS && fabs(accumulatedDeg) >= targetDeg) break;
    if (elapsed >= capMs) {
      Serial.println("[TURN] Gyro angle target not reached before cap — stopping anyway");
      break;
    }
    delay(5);
  }

  stopMotors();
  return accumulatedDeg;
}

// ── WIFI DIAGNOSTIC ──────────────────────────────────────────────────────────
void scanAndPrintNetworks() {
  Serial.println("\n[SCAN] Scanning for WiFi networks...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  int n = WiFi.scanNetworks();
  if (n == 0) {
    Serial.println("[SCAN] No networks found at all.");
  } else {
    Serial.printf("[SCAN] %d networks found:\n", n);
    bool foundTarget = false;
    for (int i = 0; i < n; i++) {
      Serial.printf("  %2d: %-32s RSSI=%d dBm  Ch=%d\n",
                    i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i));
      if (WiFi.SSID(i) == WIFI_SSID) foundTarget = true;
    }
    if (foundTarget) {
      Serial.println("[SCAN] Target SSID found in scan — likely a password or signal issue.");
    } else {
      Serial.println("[SCAN] Target SSID NOT found — likely a 5GHz-only network (ESP32 needs 2.4GHz), or it's out of range.");
    }
  }
  Serial.println();
}

// ── WIFI HELPERS ─────────────────────────────────────────────────────────────
void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.println("WiFi lost, reconnecting...");
  WiFi.reconnect();
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 5000) delay(100);
}

// ── WEBSOCKET (robot traffic: sensor stream, commands, stop, obstacle) ──────
void sendReady() {
  webSocket.sendTXT("{\"type\":\"ready\"}");
}

void sendSensorData(float distFront, float distLeft, float distRight,
                     float accelX, float accelY, float accelZ, float gyroZ) {
  String body = "{\"type\":\"sensor\",\"distance_cm\":" + String(distFront, 1) +
                ",\"distance_left_cm\":" + String(distLeft, 1) +
                ",\"distance_right_cm\":" + String(distRight, 1) +
                ",\"accel_x\":" + String(accelX, 3) +
                ",\"accel_y\":" + String(accelY, 3) +
                ",\"accel_z\":" + String(accelZ, 3) +
                ",\"gyro_z\":" + String(gyroZ, 4) + "}";
  webSocket.sendTXT(body);
}

void sendObstacleStop(char hitSide) {
  String side = hitSide == 'L' ? "\"left\"" : hitSide == 'R' ? "\"right\"" : "null";
  webSocket.sendTXT("{\"type\":\"obstacle_stop\",\"side\":" + side + "}");
}

// Dispatches an incoming JSON frame from the server. Called from webSocket.loop()
// whenever a WStype_TEXT event arrives — may fire mid-motion (runMotion calls
// webSocket.loop() every chunk), so this only ever sets flags/fields for the
// main loop() / runMotion to act on, never blocks or drives motors directly.
void onWsMessage(uint8_t* payload, size_t length) {
  StaticJsonDocument<192> doc;
  if (deserializeJson(doc, payload, length) != DeserializationError::Ok) return;

  const char* type = doc["type"] | "";
  if (strcmp(type, "command") == 0) {
    pendingCmd = String((const char*)(doc["cmd"] | "none"));
    pendingDurationMs = doc["duration_ms"] | 500;
    motorSpeedLeftPct = doc["left_pct"] | motorSpeedLeftPct;    // keep last known value if missing
    motorSpeedRightPct = doc["right_pct"] | motorSpeedRightPct;
    newCommandAvailable = true;
  } else if (strcmp(type, "stop") == 0) {
    stopRequested = true;
  }
}

void onWsEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.println("[WS] connected");
      sendReady();
      break;
    case WStype_DISCONNECTED:
      Serial.println("[WS] disconnected");
      break;
    case WStype_TEXT:
      onWsMessage(payload, length);
      break;
    default:
      break;
  }
}

// ── HTTP (calibration endpoints only — not latency-sensitive) ──────────────
int httpRequest(const String& path, bool isPost, const String& body, int timeoutMs, String& response) {
  ensureWifi();
  if (WiFi.status() != WL_CONNECTED) return -1;

  HTTPClient http;
  http.begin(String(SERVER_URL) + path);
  http.setTimeout(timeoutMs);

  int code;
  if (isPost) {
    http.addHeader("Content-Type", "application/json");
    code = http.POST(body);
  } else {
    code = http.GET();
  }
  if (code == 200) response = http.getString();
  http.end();
  return code;
}

// Reports a just-completed turn/drive's commanded target vs. what the sensors
// measured, so it can be paired on the dashboard with a real manual
// measurement and logged for calibrate_fit.py. Fire-and-forget — a dropped
// report just means one fewer calibration sample, not a functional problem.
// Angle and distance fields are mutually exclusive per row (whichever isn't
// applicable is sent as JSON null, not 0, so it isn't mistaken for a real
// zero reading downstream).
void postCalibReportInternal(const String& testType, bool hasDeg, float commandedDeg, float gyroDeg,
                              bool hasCm, float commandedCm, float accelDistanceCm,
                              int leftPct, int rightPct) {
  String body = "{\"test_type\":\"" + testType + "\","
                "\"commanded_deg\":" + (hasDeg ? String(commandedDeg, 1) : "null") + ","
                "\"gyro_deg\":" + (hasDeg ? String(gyroDeg, 1) : "null") + ","
                "\"commanded_cm\":" + (hasCm ? String(commandedCm, 1) : "null") + ","
                "\"accel_distance_cm\":" + (hasCm ? String(accelDistanceCm, 1) : "null") + ","
                "\"motor_left_pct\":" + String(leftPct) + ","
                "\"motor_right_pct\":" + String(rightPct) + "}";
  String response;
  int code = httpRequest("/calibrate/report", true, body, 500, response);
  if (code != 200) Serial.printf("[CALIB] /calibrate/report failed, HTTP %d\n", code);
}

void postCalibReportTurn(const String& testType, float commandedDeg, float gyroDeg) {
  // Turns always run at fixed full PWM regardless of the speed slider — log
  // 100/100, not motorSpeedLeftPct/RightPct, since the slider isn't what
  // actually drove the motors during a turn.
  postCalibReportInternal(testType, true, commandedDeg, gyroDeg, false, 0, 0, 100, 100);
}

void postCalibReportDistance(const String& testType, float commandedCm, float accelDistanceCm) {
  postCalibReportInternal(testType, false, 0, 0, true, commandedCm, accelDistanceCm,
                           motorSpeedLeftPct, motorSpeedRightPct);
}

// ── SETUP ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN_L, OUTPUT);
  pinMode(ECHO_PIN_L, INPUT);
  pinMode(TRIG_PIN_R, OUTPUT);
  pinMode(ECHO_PIN_R, INPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA_PIN, OUTPUT); pinMode(ENB_PIN, OUTPUT);
  stopMotors();

  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0);     // wake from sleep, default clock source
  mpuOk = (Wire.endTransmission(true) == 0);
  if (mpuOk) {
    Serial.println("[MPU6050] Initialized OK");
  } else {
    Serial.println("[MPU6050] Not found — check wiring (SDA=21, SCL=22)");
  }

  scanAndPrintNetworks();   // <-- diagnostic step, run before attempting connect

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) {
    delay(500); Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[ERROR] WiFi failed — check SSID/password, band (must be 2.4GHz), or distance to router");
  } else {
    Serial.print("\nConnected! IP: ");
    Serial.println(WiFi.localIP());
  }

  webSocket.begin(SERVER_HOST, SERVER_PORT, WS_PATH);
  webSocket.onEvent(onWsEvent);
  webSocket.setReconnectInterval(3000);
}

// ── LOOP ─────────────────────────────────────────────────────────────────────
unsigned long lastSensorSendMs = 0;
const unsigned long SENSOR_SEND_INTERVAL_MS = 100;

void loop() {
  ensureWifi();
  webSocket.loop();

  unsigned long now = millis();
  if (now - lastSensorSendMs >= SENSOR_SEND_INTERVAL_MS) {
    lastSensorSendMs = now;

    // Fire sequentially with small gaps so echoes don't cross-talk between sensors.
    float distFront = readDistanceOn(TRIG_PIN, ECHO_PIN);
    delay(20);
    float distLeft = readDistanceOn(TRIG_PIN_L, ECHO_PIN_L);
    delay(20);
    float distRight = readDistanceOn(TRIG_PIN_R, ECHO_PIN_R);
    Serial.printf("Distance F=%.1f L=%.1f R=%.1f cm\n", distFront, distLeft, distRight);

    float accelX, accelY, accelZ, gyroZ;
    readImu(accelX, accelY, accelZ, gyroZ);

    sendSensorData(distFront, distLeft, distRight, accelX, accelY, accelZ, gyroZ);
  }

  if (newCommandAvailable) {
    newCommandAvailable = false;
    String cmd = pendingCmd;
    int duration = pendingDurationMs;

    if (cmd == "forward") {
      Serial.println("CMD: forward");
      char hitSide = 0;
      bool completed;
      float accelDistanceCm;
      driveForward(duration, hitSide, completed, accelDistanceCm);
      if (hitSide) sendObstacleStop(hitSide);
      if (completed) {
        float commandedCm = (duration / (float)FORWARD_MS) * CELL_SIZE_CM;
        postCalibReportDistance("forward", commandedCm, accelDistanceCm);
      }
    } else if (cmd == "reverse") {
      Serial.println("CMD: reverse");
      bool completed;
      float accelDistanceCm;
      driveReverse(duration, completed, accelDistanceCm);
      if (completed) {
        float commandedCm = (duration / (float)FORWARD_MS) * CELL_SIZE_CM;
        postCalibReportDistance("reverse", commandedCm, accelDistanceCm);
      }
    } else if (cmd == "left") {
      Serial.println("CMD: left 90°");
      float gyroDeg = turnByAngle(true, 90.0, TURN_90_MS);
      postCalibReportTurn("left", 90.0, gyroDeg);
    } else if (cmd == "right") {
      Serial.println("CMD: right 90°");
      float gyroDeg = turnByAngle(false, 90.0, TURN_90_MS);
      postCalibReportTurn("right", 90.0, gyroDeg);
    } else if (cmd == "uturn") {
      Serial.println("CMD: U-turn 180°");
      float gyroDeg = turnByAngle(false, 180.0, TURN_90_MS * 2);
      postCalibReportTurn("uturn", 180.0, gyroDeg);
    } else if (cmd == "calib_left") {
      // Turn calibration probe: spins for the server-supplied duration_ms
      // directly instead of the fixed TURN_90_MS, so real turn angle can be
      // measured against arbitrary durations without reflashing per guess.
      Serial.printf("CMD: calib left %dms\n", duration);
      turnLeft(duration);
    } else if (cmd == "calib_right") {
      Serial.printf("CMD: calib right %dms\n", duration);
      turnRight(duration);
    } else if (cmd == "stop") {
      stopMotors();
    }

    sendReady();
  }
}
