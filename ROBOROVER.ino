/*
 * ROBOROVER PRO - STABLE VERSION (Final)
 * 🚀 Rectified: Single-read sensor logic, BLE safety, and responsive PID
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>

// --- PIN MAPPING ---
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

// --- BLE CONFIG ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pChar = NULL;
bool deviceConnected = false;

// --- ROBOT STATE ---
enum Mode { MANUAL, AUTONOMOUS };
Mode currentMode = MANUAL;
Servo myServo;

int baseSpeed = 100, maxSpeed = 200;
float Kp = 8.0, Ki = 0.01, Kd = 3.5;
int lastError = 0;
float integral = 0;
const int blackThreshold = 1500;
const int obstacleThreshold = 15;

float curL = 0, curR = 0;
int tarL = 0, tarR = 0;
char lastBTCommand = 'S';
long currentDistance = 999;
unsigned long lastPID = 0, lastNotify = 0, lastSensing = 0;
bool obstacleLocked = false;

// --- MOTOR DRIVE ---
void driveMotorsDirect(int left, int right) {
    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);
    digitalWrite(IN1, (left >= 0) ? HIGH : LOW);
    digitalWrite(IN2, (left >= 0) ? LOW : HIGH);
    ledcWrite(ENA, abs(left));
    digitalWrite(IN3, (right >= 0) ? HIGH : LOW);
    digitalWrite(IN4, (right >= 0) ? LOW : HIGH);
    ledcWrite(ENB, abs(right));
}

void ramp() {
    const float STEP = 6.0;
    if (curL < tarL) curL += STEP; else if (curL > tarL) curL -= STEP;
    if (curR < tarR) curR += STEP; else if (curR > tarR) curR -= STEP;
    driveMotorsDirect((int)curL, (int)curR);
}

void stopRobot() {
    tarL = 0; tarR = 0; curL = 0; curR = 0;
    driveMotorsDirect(0, 0); lastBTCommand = 'S';
}

// --- SENSING ---
long getDistance() {
    digitalWrite(TRIG, 0); delayMicroseconds(5);
    digitalWrite(TRIG, 1); delayMicroseconds(10);
    digitalWrite(TRIG, 0);
    long d = pulseIn(ECHO, 1, 25000); 
    return (d <= 0) ? 999 : d * 0.034 / 2;
}

int readLine() {
    int s2 = analogRead(S2), s3 = analogRead(S3), s4 = analogRead(S4);
    bool d2 = s2 > blackThreshold, d3 = s3 > blackThreshold, d4 = s4 > blackThreshold;
    if (!d2 && !d3 && !d4) return -2000;
    long val2 = d2 ? -1000 : 0, val3 = d3 ? 0 : 0, val4 = d4 ? 1000 : 0;
    int count = (d2?1:0) + (d3?1:0) + (d4?1:0);
    return (val2 + val3 + val4) / count;
}

// --- AVOIDANCE ---
void avoidObstacle() {
    stopRobot();
    pChar->setValue("MSG:SCANNING PATH"); pChar->notify();
    myServo.write(150); delay(700); long dl = getDistance();
    myServo.write(30); delay(700); long dr = getDistance();
    myServo.write(90); delay(400);

    int dir = (dl >= dr) ? 1 : -1;
    if (dl < 15 && dr < 15) {
        tarL = -80; tarR = -80; for(int i=0;i<50;i++){ramp();delay(10);}
        return;
    }

    pChar->setValue("MSG:BYPASSING..."); pChar->notify();
    tarL = -180 * dir; tarR = 180 * dir; for(int i=0;i<60;i++){ramp();delay(10);} 
    tarL = baseSpeed; tarR = baseSpeed; for(int i=0;i<100;i++){ramp();delay(10);}
    tarL = 180 * dir; tarR = -180 * dir; for(int i=0;i<60;i++){ramp();delay(10);}

    pChar->setValue("MSG:RE-ACQUIRING"); pChar->notify();
    unsigned long start = millis();
    while (readLine() == -2000 && (millis() - start < 4000)) {
        tarL=baseSpeed-30; tarR=baseSpeed-30; ramp(); delay(10);
    }
}

// --- MAIN LOOP ---
void runManual() {
    if (currentDistance <= obstacleThreshold && lastBTCommand != 'B') {
        if (!obstacleLocked) { stopRobot(); pChar->setValue("MSG:OBSTACLE!"); pChar->notify(); obstacleLocked = true; }
    } else {
        obstacleLocked = false;
        if (lastBTCommand == 'F') { tarL = baseSpeed; tarR = baseSpeed; }
        else if (lastBTCommand == 'B') { tarL = -baseSpeed; tarR = -baseSpeed; }
        else if (lastBTCommand == 'L') { tarL = baseSpeed; tarR = -baseSpeed; }
        else if (lastBTCommand == 'R') { tarL = -baseSpeed; tarR = baseSpeed; }
        else if (lastBTCommand == 'S') { stopRobot(); }
    }
    ramp();
}

void runAuto() {
    if (currentDistance <= obstacleThreshold) {
        avoidObstacle();
    } else {
        if (millis() - lastPID > 20) {
            lastPID = millis();
            int pos = readLine();
            if (pos == -2000) {
                // Lost line: spin faster to find it
                if (lastError > 0) { tarL = 110; tarR = -110; } 
                else { tarL = -110; tarR = 110; }
            } else {
                integral = constrain(integral + pos, -1000, 1000);
                float adj = (Kp * pos / 100.0) + (Ki * integral) + (Kd * (pos - lastError));
                lastError = pos;
                tarL = baseSpeed - (int)adj; tarR = baseSpeed + (int)adj;
            }
            ramp();
        }
    }
}

// --- BLE ---
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
              stopRobot(); 
              integral = 0; 
              Serial.println(">>> Mode: AUTONOMOUS"); 
            }
            else if (cmd == 'M') { 
              currentMode = MANUAL; 
              stopRobot(); 
              Serial.println(">>> Mode: MANUAL"); 
            }
            else lastBTCommand = cmd;
        }
    }
};

void setup() {
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
    myServo.attach(SERVO_PIN);
    ledcAttach(ENA, 1200, 8); ledcAttach(ENB, 1200, 8);
    BLEDevice::init("ROBOROVER_BLE");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCB());
    BLEService *pSvc = pServer->createService(SERVICE_UUID);
    pChar = pSvc->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    pChar->setCallbacks(new CharCB());
    pChar->addDescriptor(new BLE2902());
    pSvc->start();
    BLEDevice::startAdvertising();
    stopRobot();
}

void loop() {
    if (millis() - lastSensing > 50) {
        currentDistance = getDistance();
        lastSensing = millis();
    }
    if (currentMode == AUTONOMOUS) runAuto();
    else runManual();
    if (millis() - lastNotify > 500) {
        pChar->setValue((String(currentMode == MANUAL ? "M" : "A") + "," + String(currentDistance)).c_str());
        pChar->notify();
        lastNotify = millis();
    }
}
