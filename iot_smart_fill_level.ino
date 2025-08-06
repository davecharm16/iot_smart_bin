#include <WiFi.h>
#include <Wire.h>
#include <Arduino.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <time.h>

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
FirebaseJson json;

#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#define FIREBASE_PROJECT_ID ""
#define FIREBASE_API_KEY ""
#define FIREBASE_DATABASE ""
#define USER_EMAIL ""
#define USER_PASSWORD ""

#define BIN_ID "smart_bin_001"
#define BIN_HEIGHT_CM 41
#define SENSOR_OFFSET_CM 7
#define ULTRASONIC_1_TRIG_PIN 5
#define ULTRASONIC_1_ECHO_PIN 18
#define ULTRASONIC_2_TRIG_PIN 16
#define ULTRASONIC_2_ECHO_PIN 17
#define ULTRASONIC_3_TRIG_PIN 25
#define ULTRASONIC_3_ECHO_PIN 26

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 300000;  // 5 minutes

// Track notification states
bool metalNotified = false;
bool wetNotified = false;
bool dryNotified = false;

class UltraSonicSensor {
public:
  UltraSonicSensor(int trigPin, int echoPin, int binHeightCm, String materialType)
    : trigPin(trigPin), echoPin(echoPin), binHeightCm(binHeightCm), materialType(materialType) {}

  void begin() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }

  int getDistanceCm() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    return duration * 0.0343 / 2;
  }

  int getFillLevelPercentage() {
    int distance = getDistanceCm();
    if (distance <= SENSOR_OFFSET_CM) return 100;
    if (distance <= 0) return 0;

    distance -= SENSOR_OFFSET_CM;
    int fillLevelPercentage = ((binHeightCm - distance) * 100) / binHeightCm;
    return constrain(fillLevelPercentage, 0, 100);
  }

  String getMaterialType() {
    return materialType;
  }

private:
  int trigPin;
  int echoPin;
  int binHeightCm;
  String materialType;
};

// NTP Time Syncs
void updateTimeFromNTP() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("⏳ Waiting for NTP...");
  time_t now = time(nullptr);
  while (now < 100000) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("\n✅ NTP Time Synced!");
}

String getCurrentTimeFormatted() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("⚠️ Failed to get time");
    return "1970-01-01T00:00:00Z";
  }
  char timeString[25];
  strftime(timeString, sizeof(timeString), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(timeString);
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nWiFi Connected: " + WiFi.localIP().toString());

  config.api_key = FIREBASE_API_KEY;
  config.database_url = FIREBASE_DATABASE;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  Firebase.signUp(&config, &auth, "", "");
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

String generateRandomId(int length = 16) {
  static const char alphanum[] =
    "0123456789"
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz";
  String randomId;
  randomSeed(millis());
  for (int i = 0; i < length; ++i) {
    randomId += alphanum[random(0, sizeof(alphanum) - 1)];
  }
  return randomId;
}

void sendFillLevelData(const String& collectionPath, const String& documentID, const String& materialType, int percentage) {
  FirebaseJson data;
  FirebaseJson documentData;
  String formattedTime = getCurrentTimeFormatted();

  documentData.set("bin/stringValue", BIN_ID);
  documentData.set("bin_type/stringValue", materialType);
  documentData.set("percentage/integerValue", percentage);
  documentData.set("timestamp/timestampValue", formattedTime);

  data.set("fields", documentData);

  String fullPath = collectionPath + "/" + documentID;
  Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", fullPath, data.raw());
}

void sendNotification(const String& binType) {
  FirebaseJson data;
  FirebaseJson documentData;

  String formattedTime = getCurrentTimeFormatted();
  String title = String(BIN_ID) + "(" + binType + ") is already full";

  documentData.set("title/stringValue", title);
  documentData.set("timestamp/timestampValue", formattedTime);
  documentData.set("bin_id/stringValue", BIN_ID);
  documentData.set("fill_id/stringValue", generateRandomId());
  documentData.set("isRead/booleanValue", false);

  data.set("fields", documentData);
  String fullPath = "notifications/" + generateRandomId();

  if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", fullPath, data.raw())) {
    Serial.println("Notification sent: " + title);
  } else {
    Serial.println("Failed to send notification: " + fbdo.errorReason());
  }
}

UltraSonicSensor myUltraSonicSensor1(ULTRASONIC_1_TRIG_PIN, ULTRASONIC_1_ECHO_PIN, BIN_HEIGHT_CM, "Metal");
UltraSonicSensor myUltraSonicSensor2(ULTRASONIC_2_TRIG_PIN, ULTRASONIC_2_ECHO_PIN, BIN_HEIGHT_CM, "Wet");
UltraSonicSensor myUltraSonicSensor3(ULTRASONIC_3_TRIG_PIN, ULTRASONIC_3_ECHO_PIN, BIN_HEIGHT_CM, "Dry");

void setup() {
  Serial.begin(115200);
  myUltraSonicSensor1.begin();
  myUltraSonicSensor2.begin();
  myUltraSonicSensor3.begin();
  initWiFi();
  updateTimeFromNTP();
}

void loop() {
  int fillPercentage1 = myUltraSonicSensor1.getFillLevelPercentage();
  int fillPercentage2 = myUltraSonicSensor2.getFillLevelPercentage();
  int fillPercentage3 = myUltraSonicSensor3.getFillLevelPercentage();

  // Immediate Notifications when bin is full
  if (fillPercentage1 >= 90 && !metalNotified) {
    sendNotification("Metal");
    sendFillLevelData("fill_level_data", generateRandomId(), "Metal", fillPercentage1);
    metalNotified = true;
  }
  if (fillPercentage1 < 90) metalNotified = false;

  if (fillPercentage2 >= 90 && !wetNotified) {
    sendNotification("Wet");
    sendFillLevelData("fill_level_data", generateRandomId(), "Wet", fillPercentage2);
    wetNotified = true;
  }
  if (fillPercentage2 < 90) wetNotified = false;

  if (fillPercentage3 >= 90 && !dryNotified) {
    sendNotification("Dry");
    sendFillLevelData("fill_level_data", generateRandomId(), "Dry", fillPercentage3);
    dryNotified = true;
  }
  if (fillPercentage3 < 90) dryNotified = false;

  // Send Fill Levels Every 5 Min
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= sendInterval) {
    lastSendTime = currentTime;

    sendFillLevelData("fill_level_data", generateRandomId(), "Metal", fillPercentage1);
    delay(300);
    sendFillLevelData("fill_level_data", generateRandomId(), "Wet", fillPercentage2);
    delay(300);
    sendFillLevelData("fill_level_data", generateRandomId(), "Dry", fillPercentage3);
    delay(300);

    Serial.println("Fill level data sent");
  }
}
