#include <LiquidCrystal.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// *Pin Assignments*
const int alcoholSensor = A4;
const int buzzer = 13;
const int ultrasonicTrig = 12;
const int ultrasonicEcho = 11;
const int ledAlert = A5;
const int pulseSensor = A1;

// *Motor Driver Pins (L298N)*
const int motorEnable = 9;
const int motorInput1 = 8;
const int motorInput2 = 7;

// *Servo Motor Pin*
Servo myServo;
const int servoPin = 3;

// *LCD Setup (RS, E, D4, D5, D6, D7)*
LiquidCrystal lcd(2, 4, A0, A1, A2, A3);

// *GSM & GPS Module Communication Pins*
SoftwareSerial sim800l(10, 5);  // TX, RX for SIM800L
SoftwareSerial gpsSerial(6, 7); // TX, RX for GPS module

TinyGPSPlus gps;

// *System State Variables*
bool drowsyDetected = false;
bool alcoholDetected = false;
bool obstacleDetected = false;
bool abnormalPulseDetected = false;
unsigned long drowsinessStartTime = 0;
unsigned long wheelsStopTime = 0;
int currentMotorSpeed = 255;
bool ledBlinking = false;
unsigned long lastLEDToggleTime = 0;
bool ledState = true;

// *Pulse Sensor Variables*
int pulseValue = 0;
unsigned long lastPulseCheck = 0;
const int pulseThreshold = 600;  // Adjust based on testing

void setup() {
  Serial.begin(9600);
  sim800l.begin(9600);
  gpsSerial.begin(9600);

  pinMode(buzzer, OUTPUT);
  pinMode(ledAlert, OUTPUT);
  pinMode(ultrasonicTrig, OUTPUT);
  pinMode(ultrasonicEcho, INPUT);
  pinMode(alcoholSensor, INPUT);
  pinMode(pulseSensor, INPUT);

  pinMode(motorEnable, OUTPUT);
  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);

  myServo.attach(servoPin);
  lcd.begin(16, 2);

  showNormalState();
  digitalWrite(ledAlert, HIGH);
  digitalWrite(buzzer, LOW);
  moveWheels(true);
  moveServo(true);
}

void loop() {
  int alcoholLevel = analogRead(alcoholSensor);
  pulseValue = analogRead(pulseSensor);

  // Ultrasonic sensor distance
  digitalWrite(ultrasonicTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTrig, LOW);
  long duration = pulseIn(ultrasonicEcho, HIGH);
  int distance = duration * 0.034 / 2;

  // Alcohol detection
  if (alcoholLevel > 500 && !alcoholDetected) {
    alcoholDetected = true;
    showAlcoholAlert();
    sendAlert("ALCOHOL DETECTED!");
    reduceMotorSpeedGradually();
    stopServo();
  } else {
    alcoholDetected = false;
  }

  // Drowsiness detection via serial
  if (Serial.available() > 0) {
    char received = Serial.read();
    if (received == 'D' && !drowsyDetected) {
      drowsyDetected = true;
      drowsinessStartTime = millis();
      showDrowsinessAlert();
      digitalWrite(buzzer, HIGH);
      sendAlert("DROWSINESS DETECTED!");
    } else if (received == 'N') {
      drowsyDetected = false;
      stopBlinkingLED();
      stopBuzzer();
      showNormalState();
    }
  }

  // Obstacle detection
  if (distance <= 5 && !obstacleDetected) {
    obstacleDetected = true;
    showObstacleAlert();
    stopServo();
    delay(1000);
  } else {
    obstacleDetected = false;
  }

  // Pulse Sensor Monitoring
  if (millis() - lastPulseCheck > 1000) {
    lastPulseCheck = millis();
    if (pulseValue < 300 || pulseValue > pulseThreshold) {  // Abnormal pulse
      if (!abnormalPulseDetected) {
        abnormalPulseDetected = true;
        sendAlert("ABNORMAL HEART RATE DETECTED!");
        showPulseAlert();
      }
    } else {
      abnormalPulseDetected = false;
    }
  }

  // Drowsiness + Obstacle
  if (drowsyDetected && obstacleDetected) {
    stopWheels();
    stopServo();
    blinkLED(true);
  }

  // Handle LED blinking
  if ((drowsyDetected || obstacleDetected) && (millis() - wheelsStopTime < 5000 || drowsyDetected)) {
    blinkLED(true);
  } else if (!drowsyDetected) {
    stopBlinkingLED();
  }

  // Normal condition
  if (!drowsyDetected && !alcoholDetected && !obstacleDetected && !abnormalPulseDetected) {
    moveWheels(true);
    moveServo(true);
    showNormalState();
  }

  delay(100);
}

// -------------------- Helper Functions -----------------------

void showNormalState() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Everything Normal");
  lcd.setCursor(0, 1);
  lcd.print("Drive Safe...");
  digitalWrite(ledAlert, HIGH);
}

void showDrowsinessAlert() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DROWSY ALERT!");
  lcd.setCursor(0, 1);
  lcd.print("WAKE UP!");
  blinkLED(true);
  digitalWrite(buzzer, HIGH);
  stopWheels();
}

void showAlcoholAlert() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ALCOHOL");
  lcd.setCursor(0, 1);
  lcd.print("DETECTED!");
  digitalWrite(ledAlert, HIGH);
}

void showObstacleAlert() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Obstacle Ahead!");
  blinkLED(true);
}

void showPulseAlert() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Abnormal Heart");
  lcd.setCursor(0, 1);
  lcd.print("Rate Detected!");
}

void blinkLED(bool start) {
  if (start && millis() - lastLEDToggleTime > 300) {
    ledState = !ledState;
    digitalWrite(ledAlert, ledState ? HIGH : LOW);
    lastLEDToggleTime = millis();
  }
}

void stopBlinkingLED() {
  digitalWrite(ledAlert, HIGH);
}

void stopBuzzer() {
  digitalWrite(buzzer, LOW);
}

void moveWheels(bool forward) {
  currentMotorSpeed = 255;
  analogWrite(motorEnable, currentMotorSpeed);
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
}

void moveServo(bool active) {
  if (active) {
    myServo.write(90);
  }
}

void stopWheels() {
  analogWrite(motorEnable, 0);
  currentMotorSpeed = 0;
}

void stopServo() {
  myServo.write(0);
}

void reduceMotorSpeedGradually() {
  for (int speed = currentMotorSpeed; speed >= 0; speed -= 50) {
    analogWrite(motorEnable, speed);
    delay(300);
  }
  currentMotorSpeed = 0;
}

// Send alert via SIM800L
void sendAlert(String message) {
  sim800l.println("AT+CMGF=1");
  delay(100);
  sim800l.println("AT+CMGS=\"+919986762727\""); // Replace with actual number
  delay(100);
  sim800l.print(message);
  delay(100);
  sim800l.write(26);
}
