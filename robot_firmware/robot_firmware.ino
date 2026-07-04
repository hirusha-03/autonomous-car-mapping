/*
  Autonomous Exploration Robot — ESP32 Firmware
  Hardware:
    HC-SR04 front (physically mounted at rear of chassis): TRIG=GPIO18, ECHO=GPIO19
    HC-SR04 left  (~30deg off front): TRIG=GPIO4,  ECHO=GPIO5
    HC-SR04 right (~30deg off front): TRIG=GPIO15, ECHO=GPIO13
    L298N: IN1=GPIO25, IN2=GPIO26, IN3=GPIO27, IN4=GPIO14
    NOTE: ENA/ENB are jumper-capped (always full speed, no PWM control).
    KNOWN ISSUE (unresolved, flagged for later): robot drifts left on
    driveForward — left wheel spins slower than right. Needs ENA/ENB wired
    to ESP32 PWM pins + per-side speed trim to fix; deferred for now.
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ── CONFIG ───────────────────────────────────────────────────────────────────
const char* WIFI_SSID     = "Hirusha’s iPhone";
const char* WIFI_PASSWORD = "HnH@141414";
const char* SERVER_URL    = "http://172.20.10.2:8000";

// Calibration — tune these on the real robot
// TURN_90_MS was measured doing a full 360 instead of 90 at the old value
// (650ms) — hardware-tested result: 650ms ≈ 360°, so 90° ≈ 650/4.
const int TURN_90_MS  = 163;
const int FORWARD_MS  = 800;

// Motion delay() is split into chunks this long, polling /stop_flag (and,
// during driveForward, the front sensor) between each chunk — so a manual
// stop or an obstacle can abort a drive/turn already in progress instead of
// waiting out the full duration_ms blind.
const int STOP_POLL_CHUNK_MS = 150;

// Forward drive self-aborts if the front sensor reads closer than this
// mid-motion. Independent of the server's occupancy-grid obstacle ratio
// (sensorsweep.py) — this is purely a physical collision-avoidance cutoff,
// intentionally tighter than one grid cell (30cm) so it trips before contact.
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

// Requires ESP32 Arduino core >=3.0 (analogWrite is PWM-backed there; on
// older cores use ledcAttach/ledcWrite instead).
// Independent per-side rather than one speed + a fixed trim: measured drift
// direction wasn't consistent across test runs (see ai_context/INDEX.md
// Hardware Calibration Log), so a single fixed-direction correction would
// assume a bias that doesn't always hold — these are user-adjustable instead.
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

// Runs the motion for `ms` in STOP_POLL_CHUNK_MS steps, checking /stop_flag
// between each so a manual stop can cut it short. If checkObstacle is true
// (forward drive only), also checks the front sensor between chunks and
// self-aborts before contact — this is what closes the "blind for 800ms per
// cell" gap: previously the sensor was only read once, before the drive
// started, so anything that appeared mid-drive went undetected until impact.
// obstacleHit (if non-null) reports whether the obstacle check tripped.
bool runMotion(int ms, bool checkObstacle = false, bool* obstacleHit = nullptr) {
  bool hit = false;
  int remaining = ms;

  while (remaining > 0) {
    int chunk = remaining < STOP_POLL_CHUNK_MS ? remaining : STOP_POLL_CHUNK_MS;
    delay(chunk);
    remaining -= chunk;
    if (remaining <= 0) break;

    if (checkObstacle && readDistance() < SAFETY_STOP_CM) {
      Serial.println("[SAFETY] Obstacle within stop distance — aborting motion early");
      hit = true;
      break;
    }
    if (checkStopRequestedForward()) {
      Serial.println("[STOP] Manual stop — aborting motion early");
      break;
    }
  }

  stopMotors();
  if (obstacleHit) *obstacleHit = hit;
  return !hit;
}

void driveForward(int ms, bool& obstacleHit) {
  // Motor polarity swapped: sensorsweep.py assumes the HC-SR04 faces the
  // direction of travel (it marks cells ahead of robot.direction as sensed).
  // The old pin pattern drove the robot away from the sensor end, so the
  // sensor was actually reading behind the robot. Swapping polarity here
  // makes "forward" drive toward the sensor end, matching that assumption.
  setMotorPins(LOW, HIGH, LOW, HIGH);
  runMotion(ms, /*checkObstacle=*/true, &obstacleHit);
}

void driveReverse(int ms) {
  // Opposite polarity of driveForward — drives away from the sensor end.
  // No rear sensor, so no live obstacle check here.
  setMotorPins(HIGH, LOW, HIGH, LOW);
  runMotion(ms);
}

void turnLeft(int ms) {
  setMotorPins(LOW, HIGH, HIGH, LOW);
  runMotion(ms);
}

void turnRight(int ms) {
  setMotorPins(HIGH, LOW, LOW, HIGH);
  runMotion(ms);
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

// Shared HTTP helper — all endpoint calls (sensor push, command poll, stop
// poll, obstacle report) are a GET or POST to SERVER_URL+path with a JSON
// body, differing only in timeout and what they do with the response.
// Returns the HTTP status code, or -1 if WiFi wasn't connected; on success
// the response body is written to `response`.
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

void postSensorData(float distFront, float distLeft, float distRight) {
  String body = "{\"distance_cm\":" + String(distFront, 1) +
                ",\"distance_left_cm\":" + String(distLeft, 1) +
                ",\"distance_right_cm\":" + String(distRight, 1) + "}";
  String response;
  int code = httpRequest("/sensor_data", true, body, 1000, response);
  if (code != 200) Serial.printf("[SENSOR] POST failed, HTTP %d\n", code);
  else Serial.printf("[SENSOR] F=%.1f L=%.1f R=%.1f cm -> OK\n", distFront, distLeft, distRight);
}

void reportObstacleStop() {
  String response;
  int code = httpRequest("/obstacle_stop", true, "{}", 500, response);
  if (code != 200) Serial.printf("[SAFETY] /obstacle_stop report failed, HTTP %d\n", code);
}

bool checkStopRequestedForward() {
  String response;
  int code = httpRequest("/stop_flag", false, "", 300, response);  // short timeout — runs between motion chunks
  if (code != 200) return false;

  StaticJsonDocument<64> doc;
  if (deserializeJson(doc, response) != DeserializationError::Ok) return false;
  return doc["stop"] | false;
}

String fetchCommand(int& out_duration) {
  String response;
  int code = httpRequest("/command", false, "", 1000, response);
  if (code != 200) { Serial.printf("[CMD] GET failed, HTTP %d\n", code); return "none"; }

  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, response) != DeserializationError::Ok) return "none";

  out_duration = doc["duration_ms"] | 500;
  motorSpeedLeftPct = doc["left_pct"] | motorSpeedLeftPct;    // keep last known value if missing
  motorSpeedRightPct = doc["right_pct"] | motorSpeedRightPct;
  return doc["cmd"] | "none";
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
}

// ── LOOP ─────────────────────────────────────────────────────────────────────
void loop() {
  // Fire sequentially with small gaps so echoes don't cross-talk between sensors.
  float distFront = readDistanceOn(TRIG_PIN, ECHO_PIN);
  delay(20);
  float distLeft = readDistanceOn(TRIG_PIN_L, ECHO_PIN_L);
  delay(20);
  float distRight = readDistanceOn(TRIG_PIN_R, ECHO_PIN_R);
  Serial.printf("Distance F=%.1f L=%.1f R=%.1f cm\n", distFront, distLeft, distRight);
  postSensorData(distFront, distLeft, distRight);

  int duration = 500;
  String cmd = fetchCommand(duration);

  if (cmd == "forward") {
    Serial.println("CMD: forward");
    bool obstacleHit;
    driveForward(duration, obstacleHit);
    if (obstacleHit) reportObstacleStop();
  } else if (cmd == "reverse") {
    Serial.println("CMD: reverse");
    driveReverse(duration);
  } else if (cmd == "left") {
    Serial.println("CMD: left 90°");
    turnLeft(TURN_90_MS);
  } else if (cmd == "right") {
    Serial.println("CMD: right 90°");
    turnRight(TURN_90_MS);
  } else if (cmd == "uturn") {
    Serial.println("CMD: U-turn 180°");
    turnRight(TURN_90_MS * 2);
  } else if (cmd == "stop") {
    stopMotors();
  }

  delay(100);
}
