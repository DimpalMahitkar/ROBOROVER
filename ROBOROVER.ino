/*
 * ROBOROVER - PRO BLE VERSION (Web Dashboard Compatible)
 * Standard partitions: BLE takes space, so we keep the logic lean.
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>

// PIN DEFINITIONS
#define ENA 13
#define IN1 14
#define IN2 27
#define ENB 12
#define IN3 26
#define IN4 25
#define S2 33
#define S3 34
#define S4 35
#define TRIG 5
#define ECHO 18
#define SERVO_PIN 19 

// BLE SERVICE & CHAR
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;

// ROBOT STATE
enum Mode { MANUAL, AUTONOMOUS };
Mode currentMode = MANUAL;
Servo myServo;

int baseSpeed = 100, maxSpeed = 200;
float Kp = 8.0, Ki = 0.01, Kd = 3.5;
int lastError = 0;
float integral = 0;
const int blackThreshold = 1000; // Lowered threshold for greater sensitivity

float curL = 0, curR = 0;
int tarL = 0, tarR = 0;
unsigned long lastPID = 0;
int servoPos = 90, sweepDir = 5;
unsigned long lastSweep = 0;

// --- MOTOR CONTROL HELPERS (Defined early to avoid scope errors) ---

void driveMotorsDirect(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  digitalWrite(IN1, left >= 0 ? HIGH : LOW);
  digitalWrite(IN2, left >= 0 ? LOW : HIGH);
  ledcWrite(ENA, abs(left));
  digitalWrite(IN3, right >= 0 ? HIGH : LOW);
  digitalWrite(IN4, right >= 0 ? LOW : HIGH);
  ledcWrite(ENB, abs(right));
}

void ramp() {
  const float RAMP_STEP = 5.0;
  if (curL < tarL) curL += RAMP_STEP; else if (curL > tarL) curL -= RAMP_STEP;
  if (curR < tarR) curR += RAMP_STEP; else if (curR > tarR) curR -= RAMP_STEP;
  driveMotorsDirect((int)curL, (int)curR);
}

void stopRobot() {
  tarL = 0; tarR = 0;
  curL = 0; curR = 0;
  driveMotorsDirect(0, 0);
}

// --- SENSOR HELPERS ---

int readLine() {
  int s2 = analogRead(S2), s3 = analogRead(S3), s4 = analogRead(S4);
  
  // Convert to Boolean Detection based on 1500 threshold
  bool d2 = s2 > blackThreshold;
  bool d3 = s3 > blackThreshold;
  bool d4 = s4 > blackThreshold;

  // If no black detected on any sensor
  if (!d2 && !d3 && !d4) return -2000;
  
  // Map detections to position values
  long val2 = d2 ? -1000 : 0;
  long val3 = d3 ? 0 : 0;
  long val4 = d4 ? 1000 : 0;
  
  int count = (d2 ? 1 : 0) + (d3 ? 1 : 0) + (d4 ? 1 : 0);
  return (val2 + val3 + val4) / count;
}

long getDist() {
  digitalWrite(TRIG, 0); delayMicroseconds(5);
  digitalWrite(TRIG, 1); delayMicroseconds(10);
  digitalWrite(TRIG, 0);
  // 30ms timeout = ~5m detection range
  long d = pulseIn(ECHO, 1, 30000); 
  return (d <= 0) ? 999 : d * 0.034 / 2;
}

// --- LOGIC MODULES ---

void avoid() {
  stopRobot();
  delay(800);
  
  // 1. Scan with high precision
  myServo.write(150); delay(700); long dl = getDist();
  myServo.write(30); delay(700); long dr = getDist();
  myServo.write(90); delay(400);

  // Decision
  int dir = (dl >= dr) ? 1 : -1; 
  
  // 2. Execute Forced Maneuver
  // Turn 90
  tarL = -180 * dir; tarR = 180 * dir; 
  for(int i=0; i<50; i++) { ramp(); delay(10); } 
  
  // Move past
  tarL = baseSpeed + 20; tarR = baseSpeed + 20; 
  for(int i=0; i<100; i++) { ramp(); delay(10); }

  // Opposite Turn to Re-align
  tarL = 180 * dir; tarR = -180 * dir; 
  for(int i=0; i<50; i++) { ramp(); delay(10); }

  // Resume Searching
  tarL = baseSpeed; tarR = baseSpeed; 
  unsigned long start = millis();
  while (readLine() == -2000 && (millis() - start < 4000)) {
    ramp();
    delay(10);
  }
}

// BLE CALLBACKS
class ServerCB: public BLEServerCallbacks {
    void onConnect(BLEServer* s) { deviceConnected = true; }
    void onDisconnect(BLEServer* s) { deviceConnected = false; BLEDevice::startAdvertising(); }
};

class CharCB: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *c) {
        String v = String(c->getValue().c_str());
        if (v.length() > 0) {
            char cmd = v[0];
            if (cmd == 'A') { 
              currentMode = AUTONOMOUS; 
              stopRobot(); // Stop when switching to Auto
            }
            else if (cmd == 'M') { 
              currentMode = MANUAL; 
              stopRobot(); // Stop when switching to Manual
            }
            else if (cmd == 'F') { tarL = baseSpeed; tarR = baseSpeed; }
            else if (cmd == 'B') { tarL = -baseSpeed; tarR = -baseSpeed; }
            else if (cmd == 'L') { tarL = baseSpeed; tarR = -baseSpeed; }
            else if (cmd == 'R') { tarL = -baseSpeed; tarR = baseSpeed; }
            else if (cmd == 'S') { stopRobot(); }
        }
    }
};

void setup() {
  BLEDevice::init("ROBOROVER_BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCB());
  BLEService *pSvc = pServer->createService(SERVICE_UUID);
  pCharacteristic = pSvc->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->setCallbacks(new CharCB());
  pCharacteristic->addDescriptor(new BLE2902());
  pSvc->start();
  BLEDevice::startAdvertising();
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  myServo.attach(SERVO_PIN);
  ledcAttach(ENA, 1200, 8); ledcAttach(ENB, 1200, 8);
  delay(1000);
}

void loop() {
  if (currentMode == MANUAL) ramp();
  else {
    if (millis() - lastSweep > 30) { servoPos += sweepDir; if (servoPos >= 115 || servoPos <= 65) sweepDir = -sweepDir; myServo.write(servoPos); lastSweep = millis(); }
    
    long d = getDist();
    
    if (d > 2 && d < 20) { // Stable detection for 2cm-20cm 
      avoid(); 
    }
    else if (millis() - lastPID > 20) {
      lastPID = millis(); 
      int pos = readLine();
      
      // Detailed tracking debug (Uncomment to see sensor health in Serial)
      // Serial.print(analogRead(S2)); Serial.print("|"); Serial.print(analogRead(S3)); Serial.print("|"); Serial.println(analogRead(S4));
      lastPID = millis(); int pos = readLine();
      if (pos == -2000) { if (lastError > 0) { tarL=90; tarR=-90; } else { tarL=-90; tarR=90; } }
      else {
        integral += pos; integral = constrain(integral, -1000, 1000);
        float adj = (Kp * pos / 100.0) + (Ki * integral) + (Kd * (pos - lastError));
        lastError = pos; 
        tarL = baseSpeed - (int)adj; tarR = baseSpeed + (int)adj;
      }
      ramp();
    }
  }
  static unsigned long lastNotify = 0;
  if (deviceConnected && millis() - lastNotify > 500) {
    pCharacteristic->setValue(((currentMode == MANUAL ? "M" : "A") + String(",") + String(getDist())).c_str());
    pCharacteristic->notify();
    lastNotify = millis();
  }
}
