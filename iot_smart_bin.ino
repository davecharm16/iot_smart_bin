#include <ESP32Servo.h>
#include <WiFi.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

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
    InductiveSensor(int pin) : _pin(pin) {
        pinMode(_pin, INPUT);
    }

    bool detectObject() override {
        return (digitalRead(_pin) == LOW); // Metal detected = LOW
    }

private:
    int _pin;
};

class WetWasteSensor {
public:
    WetWasteSensor(int sensorPin, int threshold = 500)
        : sensorPin(sensorPin), threshold(threshold) {}

    void begin() {
        pinMode(sensorPin, INPUT);
    }

    bool detectObject() {
        int moistureValue = analogRead(sensorPin);
        Serial.print("Moisture Value: ");
        Serial.println(moistureValue);
        return moistureValue < threshold; // Inverted logic since lower value means wetter
    }

    // Calibration method (similar to the previous version)
    void calibrate() {
        Serial.println("WetWasteSensor Calibration:");
        Serial.println("------------------------");

        // Dry Calibration
        Serial.println("1. Place DRY material on the sensor.");
        Serial.println("2. Press any key to continue...");
        while (!Serial.available());
        Serial.read(); // Clear the buffer

        long dryReading = analogRead(sensorPin);
        Serial.print("Dry Reading: ");
        Serial.println(dryReading);

        // Wet Calibration
        Serial.println("\n1. Place WET material on the sensor.");
        Serial.println("2. Press any key to continue...");
        while (!Serial.available());
        Serial.read(); // Clear the buffer

        long wetReading = analogRead(sensorPin);
        Serial.print("Wet Reading: ");
        Serial.println(wetReading);

        // Calculate and Set Threshold
        threshold = (dryReading + wetReading) / 2;
        Serial.print("\nNew threshold set to: ");
        Serial.println(threshold);
        Serial.println("------------------------");
    }

    void calibrateWithAveraging(){
      Serial.println("WetWasteSensor Calibration:");
      Serial.println("------------------------");

      // Dry Calibration
      Serial.println("1. Place DRY material on the sensor.");
      Serial.println("2. Press any key to continue...");
      while (!Serial.available());
      Serial.read(); // Clear the buffer

      int numReadings = 10;
      long dryReadingsTotal = 0;
      for (int i = 0; i < numReadings; i++) {
          dryReadingsTotal += analogRead(sensorPin);
          delay(100); // Short delay between readings (adjust if needed)
      }
      long dryReading = dryReadingsTotal / numReadings;
      Serial.print("Average Dry Reading: ");
      Serial.println(dryReading);

      // Wet Calibration
      Serial.println("\n1. Place WET material on the sensor.");
      Serial.println("2. Press any key to continue...");
      while (!Serial.available());
      Serial.read(); // Clear the buffer

      long wetReadingsTotal = 0;
      for (int i = 0; i < numReadings; i++) {
          wetReadingsTotal += analogRead(sensorPin);
          delay(100);
      }
      long wetReading = wetReadingsTotal / numReadings;
      Serial.print("Average Wet Reading: ");
      Serial.println(wetReading);

      // Calculate and Set Threshold
      threshold = (dryReading + wetReading) / 2;
      Serial.print("\nNew threshold set to: ");
      Serial.println(threshold);
      Serial.println("------------------------");
    }

private:
    int sensorPin;
    int threshold;
};

class InfraredSensor : public Sensor {
public:
    InfraredSensor(int pin) : _pin(pin) {
        pinMode(_pin, INPUT);
    }

    bool detectObject() override {
        return (digitalRead(_pin) == LOW); // Object detected = LOW
    }

private:
    int _pin;
};

class ESP32ServoClass {
public:
    // Constructor
    ESP32ServoClass(int pin) : servoPin(pin) {
        servo.attach(servoPin);
    }

    // Basic servo control functions
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
        for (int angle = startAngle; angle <= endAngle; angle++) {
            writeAngle(angle);
            delay(delayTime);
        }
    }

private:
    int servoPin;
    Servo servo;  
};

class UltrasonicFillLevel {
public:
    UltrasonicFillLevel(int trigPin, int echoPin, int binHeightCm) 
        : trigPin(trigPin), echoPin(echoPin), binHeightCm(binHeightCm) { }

    void begin() {
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
    }

    int getDistanceCm() {
        // Standard ultrasonic sensor distance calculation
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        long duration = pulseIn(echoPin, HIGH);
        return duration * 0.0343 / 2; // Speed of sound calculation
    }

    int getFillLevelPercentage() {
        int distance = getDistanceCm();
        if (distance <= 0) {
            return 100; // Bin is full or sensor error
        }
        int fillHeight = binHeightCm - distance;
        return (fillHeight * 100) / binHeightCm; 
    }

private:
    int trigPin;
    int echoPin;
    int binHeightCm;
};

/////////////////////////////////////////////////////////////////////////////////////////////////
//PIN DEFINITIONS

#define WIFI_SSID "HUAWEI-W8wQ"
#define WIFI_PASSWORD "wonderful" 
#define FIREBASE_PROJECT_ID "test-2ac5c"
#define FIREBASE_API_KEY "AIzaSyAOMYONHuV6hMmfVUnoEadnALvnx6Qo2nU"
#define FIREBASE_DATABASE "https://test-2ac5c-default-rtdb.asia-southeast1.firebasedatabase.app"
#define USER_EMAIL "123@gmail.com"
#define USER_PASSWORD "12345678"


#define SERVO_1_PIN 13
#define SERVO_2_PIN 12
#define INFRARED_PIN 14
#define INDUCTIVE_PIN 27
#define WET_PIN 34
#define LCDCOLUMNS 16
#define LCDROWS 2
bool signupOK = false;





//////////////////////////////////////////////////////////////////////////////////////////////////
//variable definitionsss
ESP32ServoClass myServo1(SERVO_1_PIN);
ESP32ServoClass myServo2(SERVO_2_PIN);
InductiveSensor myInductiveSensor(INDUCTIVE_PIN);
WetWasteSensor myWetSensor(WET_PIN);
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

    if(isObjectDetected) {
      if(isMetalDetected){
        return METAL;
      }
      else if (isWetDetected){
        return WET;
      }
      else{
        return DRY;
      }
    }
    else{
      return NONE;
    }

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
  }
  else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  myServo1.writeAngle(90);
  myServo2.writeAngle(90);
}
 
void setup() {
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();
	// Allow allocation of all timers
  Serial.begin(115200);
  displayMessage("Initiating WIFI Connection...", 0, true);

  initWiFi();
  Serial.print("RRSI: ");
  Serial.println(WiFi.RSSI());
  Serial.println("Setup done");
  displayMessage("Setup Done", 0, true);
  displayMessage("Calibrating...", 0, true);
  myWetSensor.calibrateWithAveraging();
  displayMessage("Calibration Done", 0, true);
  delay(1000);
}

void writeDataToFirestore(const String& collectionPath, MaterialType materialType) {
    if (Firebase.ready()) {
        FirebaseJson data;
        FirebaseJson nestedData;

        // Use correct structure for Firestore
        String materialTypeString = (materialType == METAL) ? "METAL" :
                                    (materialType == WET) ? "WET" :
                                    (materialType == DRY) ? "DRY" : "NONE";

        // Setting nested data with the correct Firestore data types
        nestedData.set("type/stringValue", materialTypeString); // Correct key for string type
        nestedData.set("timestamp/timestampValue", Firebase.getCurrentTime()); // Correct key for timestamp

        // Creating a data field that is a map containing the nested data
        data.set("fields/data/mapValue", nestedData); // 'data' is a map that contains 'nestedData'

        String fullPath = collectionPath;  // Ensure the collection path is correct

        if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", fullPath, data.raw())) {
            Serial.println("Document created successfully in Firestore!");
            Serial.println("Auto-generated Document ID: " + String(fbdo.payload().c_str()));
        } else {
            Serial.println("Failed to create document in Firestore: " + fbdo.errorReason());
        }
    } else {
        Serial.println("Firebase not ready. Check connection.");
    }
}

void sendSampleDataToFirestore(const String& collectionPath, const String& documentID) {
    if (Firebase.ready()) {
        FirebaseJson data;

        // Manually formatting the current time for Firestore
        // Ideally, you should replace this with actual time fetching and formatting
        String formattedTime = "2021-06-01T12:00:00Z";  // Example static time, replace with actual formatted time

        // Creating dummy data for the document
        FirebaseJson documentData;
        documentData.set("sensorType/stringValue", "Inductive");
        documentData.set("reading/integerValue", 42);
        documentData.set("status/stringValue", "Active");
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
    randomSeed(millis()); // Seed with milliseconds since boot for variation

    for (int i = 0; i < length; ++i) {
        randomId += alphanum[random(0, sizeof(alphanum) - 1)];
    }
    return randomId;
}


void loop() {

  lcd.print("Scanning...");
  delay(3000); 
  MaterialType detectedMaterial = detectMaterial();

  lcd.clear();

  switch (detectedMaterial) {
    case METAL:
      Serial.print('Metal Detected \n');
        lcd.print("METAL");  
        delay(2000); 
        break;
    case WET:  
        lcd.print("WET");     
        delay(2000); 
        break;
    case DRY: 
        lcd.print("DRY"); 
        delay(2000); 
        break;
    default:       
        displayMessage("Scanning...",0, true);      
        break;
  }

  // writeDataToFirestore("waste_data", detectedMaterial);
  // sendDummyDataToFirestore("test_collection");
  sendSampleDataToFirestore("sensorData", "sampleDoc");
  lcd.clear();

  
}

