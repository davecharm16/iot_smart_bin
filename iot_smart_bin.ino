#include <ESP32Servo.h>
#include <WiFi.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>



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
        return (digitalRead(_pin) == HIGH); // Metal detected = HIGH
    }

private:
    int _pin;
};

class WetSensor : public Sensor {
public:
    WetSensor(int pin) : _pin(pin) {
        pinMode(_pin, INPUT);
    }

    bool detectObject() override {
        return (digitalRead(_pin) == HIGH); // Wetness detected = HIGH
    }

private:
    int _pin;
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


/////////////////////////////////////////////////////////////////////////////////////////////////
//PIN DEFINITIONS

#define WIFI_SSID "HUAWEI-W8wQ"
#define WIFI_PASSWORD "wonderful"
#define FIREBASE_HOST "fir-app-example.firebaseio.com"
#define FIREBASE_AUTH "examplesd2asdasdasdasd2asd3asd2asd2as32das3d2as2da3"

#define SERVO_1_PIN 13
#define SERVO_2_PIN 12
#define INFRARED_PIN 14
#define INDUCTIVE_PIN 32  
#define WET_PIN 33 
#define LCDCOLUMNS 16
#define LCDROWS 2





//////////////////////////////////////////////////////////////////////////////////////////////////
//variable definitionsss
ESP32ServoClass myServo1(SERVO_1_PIN);
ESP32ServoClass myServo2(SERVO_2_PIN);
InductiveSensor myInductiveSensor(INDUCTIVE_PIN);
WetSensor myWetSensor(WET_PIN);
InfraredSensor myIRSensor(INFRARED_PIN);
LiquidCrystal_I2C lcd(0x27, LCDCOLUMNS, LCDROWS);




enum MaterialType {
    NONE,
    METAL,
    ORGANIC,
    NON_ORGANIC
};

void displayMessage(const String& message, int line = 0, bool clearFirst = true) {
    if (clearFirst) {
        lcd.clear();
    }
    lcd.setCursor(0, line);
    lcd.print(message);
}


MaterialType detectMaterial() {
    bool metalDetected = myInductiveSensor.detectObject();
    bool organicDetected = myWetSensor.detectObject();
    bool nonOrganicDetected = myIRSensor.detectObject() && !metalDetected && !organicDetected;

    if (metalDetected) {
        return METAL;
    } else if (organicDetected) {
        return ORGANIC;
    } else if (nonOrganicDetected) {
        return NON_ORGANIC;
    } else {
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
  delay(1000);
}


 
void loop() {
  lcd.setCursor(0, 0);
  // print message
  lcd.print("I love you !!!");
  delay(1000);
  // clears the display to print new message
  lcd.clear();
  // set cursor to first column, second row
  lcd.setCursor(0,1);
  lcd.print("Donnaa!!!");
  delay(1000);
  lcd.clear();
}

