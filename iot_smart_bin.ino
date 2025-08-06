#include <ESP32Servo.h>
#include <WiFi.h>
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <time.h>

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
FirebaseJson json;

/////////////////////////////////////////////////
/////////Utility Classes
class Sensor {
public:
  virtual bool detectObject() = 0;
};


class InductiveSensor : public Sensor {
public:
  InductiveSensor(int pin)
    : _pin(pin) {
    pinMode(_pin, INPUT);
  }

  bool detectObject() override {
    return (digitalRead(_pin) == LOW);
  }

private:
  int _pin;
};

class WetWasteSensor {
public:
  WetWasteSensor(int sensorPower, int sensorPin, int threshold = 400)
    : sensorPower(sensorPower), sensorPin(sensorPin), threshold(threshold) {}

  void begin() {
    pinMode(sensorPower, OUTPUT);
    digitalWrite(sensorPower, LOW);
  }

  bool detectObject() {
    digitalWrite(sensorPower, HIGH);
    int wetValue = analogRead(sensorPin);
    Serial.print("wet Value: ");
    Serial.println(wetValue);
    digitalWrite(sensorPower, LOW);
    return wetValue > threshold;
  }

private:
  int sensorPower;
  int sensorPin;
  int threshold;
};

class InfraredSensor : public Sensor {
public:
  InfraredSensor(int pin)
    : _pin(pin) {
    pinMode(_pin, INPUT);
  }

  bool detectObject() override {
    return (digitalRead(_pin) == LOW);
  }

private:
  int _pin;
};

class ESP32ServoClass {
public:
  ESP32ServoClass(int pin)
    : servoPin(pin) {
    servo.attach(servoPin);
  }

  void writeAngle(int angle) {
    servo.write(angle);
  }

  int readAngle() {
    return servo.read();
  }

  void detach() {
    servo.detach();
  }

  // Additional helper functions (customize as needed)
  void sweep(int startAngle, int endAngle, int delayTime = 15) {
    servo.attach(servoPin);
    for (int angle = startAngle; angle <= endAngle; angle++) {
      writeAngle(angle);
      delay(delayTime);
    }
    servo.detach();
  }

private:
  int servoPin;
  Servo servo;
};

/////////////////////////////////////////////////////////////////////////////////////////////////
//PIN and VARIABLE DEFINITIONS

#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#define FIREBASE_PROJECT_ID ""
#define FIREBASE_API_KEY ""
#define FIREBASE_DATABASE ""
#define USER_EMAIL ""
#define USER_PASSWORD ""
#define BIN_ID "smart_bin_001"

#define SERVO_1_PIN 13
#define SERVO_2_PIN 12
#define SERVO_3_PIN 25
#define INFRARED_PIN 14
#define INDUCTIVE_PIN 27
#define WET_POWER 17
#define WET_PIN 34
#define LCDCOLUMNS 16
#define LCDROWS 2
bool signupOK = false;

ESP32ServoClass myServo1(SERVO_1_PIN);
ESP32ServoClass myServo2(SERVO_2_PIN);
ESP32ServoClass myServo3(SERVO_3_PIN);

InductiveSensor myInductiveSensor(INDUCTIVE_PIN);
WetWasteSensor myWetSensor(WET_POWER, WET_PIN);
InfraredSensor myIRSensor(INFRARED_PIN);
LiquidCrystal_I2C lcd(0x27, LCDCOLUMNS, LCDROWS);

enum MaterialType {
  NONE,
  METAL,
  WET,
  DRY
};

void displayMessage(const String& message, int line = 0, bool clearFirst = true) {
  if (clearFirst) {
    lcd.clear();
  }
  lcd.setCursor(0, line);
  lcd.print(message);
}


MaterialType detectMaterial() {
  bool isMetalDetected = myInductiveSensor.detectObject();
  bool isWetDetected = myWetSensor.detectObject();
  bool isObjectDetected = myIRSensor.detectObject();

  if (isObjectDetected) {
    if (isMetalDetected) {
      return METAL;
    } else if (isWetDetected) {
      return WET;
    } else {
      return DRY;
    }
  } else {
    return NONE;
  }
}

void dump() {
  myServo3.sweep(140, -40, 10);
  delay(2000);
  myServo3.sweep(-40, 140, 5);
  delay(2000);
  myServo3.detach();
}


void updateTimeFromNTP() {
  configTime(0, 0, "ph.pool.ntp.org", "time.nist.gov");
  Serial.println("Waiting for time");
  while (!time(nullptr)) {
    Serial.print(".");
    delay(1000);
  }
}


// Function to get the current time in ISO8601 format
String getCurrentTimeFormatted() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return "";
  }
  char timeString[25];                                                        // Buffer to hold the time string
  strftime(timeString, sizeof(timeString), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);  // ISO8601 format
  return String(timeString);
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());

  /* Assign the api key (required) */
  config.api_key = FIREBASE_API_KEY;

  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  /* Assign the RTDB URL (required) */
  config.database_url = FIREBASE_DATABASE;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("ok");
    signupOK = true;
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback;  //see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  myServo1.writeAngle(90);
  myServo2.writeAngle(90);
  myServo3.writeAngle(120);
  myServo3.detach();
}

void sendWasteData(const String& collectionPath, const String& documentID, const String& materialType) {
  if (Firebase.ready()) {
    FirebaseJson data;

    // Manually formatting the current time for Firestore
    // Ideally, you should replace this with actual time fetching and formatting
    String formattedTime = getCurrentTimeFormatted();  // Example static time, replace with actual formatted time

    // Creating dummy data for the document
    FirebaseJson documentData;
    documentData.set("type/stringValue", materialType);
    // documentData.set("reading/integerValue", 42);
    documentData.set("bin_id/stringValue", BIN_ID);
    documentData.set("timestamp/timestampValue", formattedTime);  // Using correctly formatted time

    // Setting the document fields
    data.set("fields", documentData);

    // Constructing the full path to the document
    String fullPath = collectionPath + "/" + documentID;

    // Creating or updating the document at the specified path in Firestore
    if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", fullPath, data.raw())) {
      Serial.println("Document created/updated successfully in Firestore at: " + fullPath);
      Serial.println("Document Data: " + String(fbdo.payload().c_str()));
    } else {
      Serial.println("Failed to create/update document in Firestore: " + fbdo.errorReason());
    }
  } else {
    Serial.println("Firebase not ready. Check connection.");
  }
}

String generateRandomId(int length = 16) {
  static const char alphanum[] =
    "0123456789"
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz";

  String randomId;
  randomSeed(millis());  // Seed with milliseconds since boot for variation

  for (int i = 0; i < length; ++i) {
    randomId += alphanum[random(0, sizeof(alphanum) - 1)];
  }
  return randomId;
}


void setup() {
  lcd.init();
  lcd.backlight();
  Serial.begin(115200);
  displayMessage("Initiating WIFI Connection...", 0, true);

  initWiFi();
  Serial.print("RRSI: ");
  Serial.println(WiFi.RSSI());
  updateTimeFromNTP();
  myWetSensor.begin();
  Serial.println("Setup done");
  displayMessage("Setup Done", 0, true);

  delay(1000);
}


void loop() {
  lcd.clear();
  displayMessage("Scanning...", 0, true);

  delay(1000);
  MaterialType detectedMaterial = detectMaterial();

  switch (detectedMaterial) {
    case METAL:
      displayMessage("Metal Detected", 0, true);
      delay(1000);
      displayMessage("Bin Opening", 0, true);
      myServo2.writeAngle(180);
      delay(1000);
      dump();
      delay(3000);
      myServo2.writeAngle(90);
      sendWasteData("waste_data", generateRandomId(), "Metal");
      break;
    case WET:
      displayMessage("Wet Detected", 0, true);
      delay(1000);
      displayMessage("Bin Opening", 0, true);
      myServo1.writeAngle(180);
      delay(1000);
      dump();
      delay(2000);
      myServo1.writeAngle(90);
      sendWasteData("waste_data", generateRandomId(), "Wet");
      break;
    case DRY:
      displayMessage("Dry Detected", 0, true);
      delay(1000);
      dump();
      displayMessage("Bin Opening", 0, true);
      delay(2000);
      sendWasteData("waste_data", generateRandomId(), "Dry");
      break;
    default:
      break;
  }
}