/*
  Autonomous Exploration Robot — ESP32 Firmware
  Hardware:
    HC-SR04 (physically mounted at rear): TRIG=GPIO18, ECHO=GPIO19
    L298N: IN1=GPIO25, IN2=GPIO26, IN3=GPIO27, IN4=GPIO14
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ── CONFIG ───────────────────────────────────────────────────────────────────
const char* WIFI_SSID     = "Sagarika's A07";
const char* WIFI_PASSWORD = "qqqqqqqq";
const char* SERVER_URL    = "http://192.168.56.1:8000";

// Calibration — tune these on the real robot
const int TURN_90_MS  = 650;
const int FORWARD_MS  = 800;
const int MOTOR_SPEED = 200;

// Manual-stop responsiveness: motion delay() is split into chunks this long,
// polling /stop_flag between each so a manual stop can abort a drive/turn
// already in progress instead of waiting out the full duration_ms.
const int STOP_POLL_CHUNK_MS = 150;

// ── PIN DEFINITIONS ──────────────────────────────────────────────────────────
#define TRIG_PIN 18
#define ECHO_PIN 19
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14

// ── SENSOR ───────────────────────────────────────────────────────────────────
float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) return 400.0;
  return duration * 0.0343f / 2.0f;
}

// ── MOTORS ───────────────────────────────────────────────────────────────────
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// Runs the motion for `ms`, but checks /stop_flag every STOP_POLL_CHUNK_MS
// so a manual stop can cut it short instead of blocking for the full duration.
void runMotion(int ms) {
  int remaining = ms;
  while (remaining > 0) {
    int chunk = remaining < STOP_POLL_CHUNK_MS ? remaining : STOP_POLL_CHUNK_MS;
    delay(chunk);
    remaining -= chunk;
    if (remaining > 0 && checkStopRequested()) {
      Serial.println("[STOP] Manual stop — aborting motion early");
      break;
    }
  }
  stopMotors();
}

void driveForward(int ms) {
  // Motor polarity swapped: sensorsweep.py assumes the HC-SR04 faces the
  // direction of travel (it marks cells ahead of robot.direction as sensed).
  // The old pin pattern drove the robot away from the sensor end, so the
  // sensor was actually reading behind the robot. Swapping polarity here
  // makes "forward" drive toward the sensor end, matching that assumption.
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  runMotion(ms);
}

void turnLeft(int ms) {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  runMotion(ms);
}

void turnRight(int ms) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
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

void postSensorData(float distance) {
  ensureWifi();
  if (WiFi.status() != WL_CONNECTED) { Serial.println("[SENSOR] WiFi not connected, skipping"); return; }

  HTTPClient http;
  http.begin(String(SERVER_URL) + "/sensor_data");
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(1000);

  String body = "{\"distance_cm\":" + String(distance, 1) + "}";
  int code = http.POST(body);
  if (code != 200) Serial.printf("[SENSOR] POST failed, HTTP %d\n", code);
  else Serial.printf("[SENSOR] %.1f cm → OK\n", distance);
  http.end();
}

bool checkStopRequested() {
  if (WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  http.begin(String(SERVER_URL) + "/stop_flag");
  http.setTimeout(300);  // short — this runs between motion chunks, must stay snappy

  int code = http.GET();
  if (code != 200) { http.end(); return false; }

  String payload = http.getString();
  http.end();

  StaticJsonDocument<64> doc;
  if (deserializeJson(doc, payload) != DeserializationError::Ok) return false;

  return doc["stop"] | false;
}

String fetchCommand(int& out_duration) {
  ensureWifi();
  if (WiFi.status() != WL_CONNECTED) return "none";

  HTTPClient http;
  http.begin(String(SERVER_URL) + "/command");
  http.setTimeout(1000);

  int code = http.GET();
  if (code != 200) { Serial.printf("[CMD] GET failed, HTTP %d\n", code); http.end(); return "none"; }

  String payload = http.getString();
  http.end();

  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, payload) != DeserializationError::Ok) return "none";

  out_duration = doc["duration_ms"] | 500;
  return doc["cmd"] | "none";
}

// ── SETUP ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
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
  float dist = readDistance();
  Serial.printf("Distance: %.1f cm\n", dist);
  postSensorData(dist);

  int duration = 500;
  String cmd = fetchCommand(duration);

  if (cmd == "forward") {
    Serial.println("CMD: forward");
    driveForward(duration);
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