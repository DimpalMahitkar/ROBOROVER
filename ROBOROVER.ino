/*
 * ROBOROVER PRO - 100% RECTIFIED MASTER FIRMWARE
 * Logic: Unified Manual + PD Autonomous (with Console Messages)
 * Fixed: Scope Shadowing, Variable Overflow, and Div-by-Zero
 */

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <ESP32Servo.h>

// ---------------- PINS (Standardized) ----------------
#define ENA 13
#define IN1 14
#define IN2 27
#define ENB 12
#define IN3 26
#define IN4 25
#define S_LEFT 33
#define S_CENTER 34
#define S_RIGHT 35
#define TRIG 5
#define ECHO 18
#define SERVO_PIN 19

// ---------------- BLE CORE ----------------
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer *pServer = NULL;
BLECharacteristic *pChar = NULL;
bool deviceConnected = false;

// ---------------- ROBOT STATE ----------------
enum Mode { MANUAL, AUTONOMOUS };
Mode currentMode = MANUAL;

Servo myServo;
int servoPos = 90;
int sweepDir = 5;
unsigned long lastSweepTime = 0;
unsigned long lastNotify = 0;

// ---------------- PARAMETERS ----------------
int baseSpeed = 85;
int maxSpeed =
    200;          // Raised max speed limit so your turns are no longer capped!
float Kp = 0.22f; // Softened to prevent the initial "kick"
float Kd = 12.0f; // Increased to provide heavy damping (anti-oscillation)
int lastError = 0;
char lastCmd = 'S';
bool lastAvoidWasLeft = false;

const int blackLine = 1500;
const int deadband = 50;

// ---------------- SYSTEM MESSAGES ----------------
void notify(const char *msg) {
  if (deviceConnected && pChar) {
    pChar->setValue(msg);
    pChar->notify();
  }
}

// ---------------- MOTOR CONTROL ----------------
void setMotor(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  digitalWrite(IN1, leftSpeed >= 0 ? HIGH : LOW);
  digitalWrite(IN2, leftSpeed >= 0 ? LOW : HIGH);
  ledcWrite(ENA, abs(leftSpeed));

  digitalWrite(IN3, rightSpeed >= 0 ? HIGH : LOW);
  digitalWrite(IN4, rightSpeed >= 0 ? LOW : HIGH);
  ledcWrite(ENB, abs(rightSpeed));
}

// ---------------- AVOIDANCE LOGIC ----------------
void physicalTurnLeft() {
  // Left Motor Backward, Right Motor Forward = Pivots Left physically
  setMotor(-180, 180);
}

void physicalTurnRight() {
  // Left Motor Forward, Right Motor Backward = Pivots Right physically
  setMotor(180, -180);
}

void physicalDriveForward() { setMotor(baseSpeed + 20, baseSpeed + 20); }

void obstacleBypass() {
  setMotor(0, 0);
  delay(300);

  notify("MSG:SCAN LOOK LEFT");
  long distLeft = getBestSideClearance(true);

  notify("MSG:SCAN LOOK RIGHT");
  long distRight = getBestSideClearance(false);

  myServo.write(90);
  delay(300);

  const long clearMin = 20; // Side is clear above this distance (cm)
  const long chooseGap = 5; // Ignore tiny differences due to sonar noise
  bool leftClear = distLeft >= clearMin;
  bool rightClear = distRight >= clearMin;
  bool goLeft = false;

  // Decision order:
  // 1) If only one side is clear, choose it.
  // 2) If both are clear/blocked, choose clearly larger free side.
  // 3) If nearly equal, alternate turn direction (prevents right bias).
  if (leftClear && !rightClear) {
    goLeft = true;
  } else if (!leftClear && rightClear) {
    goLeft = false;
  } else if (distLeft > distRight + chooseGap) {
    goLeft = true;
  } else if (distRight > distLeft + chooseGap) {
    goLeft = false;
  } else {
    goLeft = !lastAvoidWasLeft;
  }

  if (goLeft) {
    notify("MSG:MOVING LEFT");
    lastAvoidWasLeft = true;

    physicalTurnLeft();
    delay(
        400); // Sharply reduced to prevent over-rotation and BLE core stalling

    physicalDriveForward();
    delay(800);

    // Take the opposite turn according to the turn it took earlier
    physicalTurnRight();
    delay(500); // Sharply reduced

  } else {
    notify("MSG:MOVING RIGHT");
    lastAvoidWasLeft = false;

    physicalTurnRight();
    delay(400); // Sharply reduced

    physicalDriveForward();
    delay(800);

    // Take the opposite turn according to the turn it took earlier
    physicalTurnLeft();
    delay(500); // Sharply reduced
  }

  // When it senses the black line again it should start to follow it
  notify("MSG:FINDING LINE");
  unsigned long startSearch = millis();
  while (millis() - startSearch < 6000) {
    if (analogRead(S_LEFT) > blackLine || analogRead(S_CENTER) > blackLine ||
        analogRead(S_RIGHT) > blackLine) {
      setMotor(0, 0); // Active brake
      delay(100);
      lastError = 0;
      notify("MSG:LINE REACQUIRED");
      break;
    }
    physicalDriveForward();
    delay(15);
  }
}

// ---------------- SENSING & LOGIC ----------------
long getDist() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long d = pulseIn(ECHO, HIGH, 20000); // 20ms timeout
  return (d == 0) ? 999 : d * 0.034 / 2;
}

long getStableDistAt(int angle) {
  myServo.write(angle);
  delay(350); // Allow servo + ultrasonic cone to settle
  return getDist();
}

long getBestSideClearance(bool leftSide) {
  // Read each side from two nearby angles to reduce chassis/self-echo bias.
  // We choose the better (larger) clearance for decision making.
  if (leftSide) {
    long d1 = getStableDistAt(125);
    long d2 = getStableDistAt(145);
    return (d1 > d2) ? d1 : d2;
  } else {
    long d1 = getStableDistAt(55);
    long d2 = getStableDistAt(35);
    return (d1 > d2) ? d1 : d2;
  }
}

int readLine() {
  // Rectified: Read and filter in one pass
  long sL = analogRead(S_LEFT), sC = analogRead(S_CENTER),
       sR = analogRead(S_RIGHT);
  sL = (sL > blackLine) ? sL : 0;
  sC = (sC > blackLine) ? sC : 0;
  sR = (sR > blackLine) ? sR : 0;

  long sum = sL + sC + sR;
  if (sum < 500)
    return -1; // User threshold for line detection

  // Rectified: Weighted average with explicit long casting to prevent overflow
  return (int)(((sL * 0L) + (sC * 1000L) + (sR * 2000L)) / sum);
}

// ---------------- BLE CALLBACKS ----------------
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *s) { deviceConnected = true; }
  void onDisconnect(BLEServer *s) {
    deviceConnected = false;
    BLEDevice::startAdvertising();
  }
};

class CharCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) {
    String data = String(c->getValue().c_str());
    if (data.length() > 0) {
      char cmd = data[0];
      if (cmd == 'A') {
        currentMode = AUTONOMOUS;
        setMotor(0, 0);
        lastError = 0;
        notify("MSG:AI ONLINE");
      } else if (cmd == 'M') {
        currentMode = MANUAL;
        setMotor(0, 0);
        lastCmd = 'S';
        notify("MSG:MANUAL LINK ACTIVE");
      } else if (cmd == 'S') {
        currentMode = MANUAL;
        setMotor(0, 0);
        lastCmd = 'S';
        notify("MSG:SYSTEM HALTED");
      } else {
        lastCmd = cmd;
      }
    }
  }
};

// ---------------- LOGIC HANDLERS ----------------
void runManual() {
  if (lastCmd == 'F')
    setMotor(baseSpeed + 25, baseSpeed + 25);
  else if (lastCmd == 'B')
    setMotor(-baseSpeed - 20, -baseSpeed - 20);
  else if (lastCmd == 'L')
    setMotor(-180, 180); // Fast Left Turn
  else if (lastCmd == 'R')
    setMotor(180, -180); // Fast Right Turn
  else
    setMotor(0, 0);
}

void runAuto() {
  long dist = getDist();
  if (dist < 15) {
    delay(20); // Small delay
    if (getDist() < 15) { // Double check to reject false noise spikes
      notify("MSG:OBSTACLE DETECTED");
      obstacleBypass();
    }
  } else {
    int pos = readLine();
    if (pos == -1) {
      // Recovery logic
      if (lastError > 0)
        setMotor(100, 35);
      else
        setMotor(35, 100);
    } else {
      int error = pos - 1000;
      if (abs(error) < deadband)
        error = 0;

      // Rectified: Balanced PD Correction
      float corr = (Kp * (float)error) + (Kd * (float)(error - lastError));
      lastError = error;
      setMotor(baseSpeed + (int)corr, baseSpeed - (int)corr);
    }
  }
}

// ---------------- SETUP & LOOP ----------------
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2400);
  myServo.write(90); // Center exactly once on boot

  ledcAttach(ENA, 1200, 8);
  ledcAttach(ENB, 1200, 8);

  BLEDevice::init("ROBOROVER_BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCB());

  BLEService *pSvc = pServer->createService(SERVICE_UUID);
  pChar = pSvc->createCharacteristic(CHARACTERISTIC_UUID,
                                     BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY);
  pChar->setCallbacks(new CharCB());
  pChar->addDescriptor(new BLE2902());

  pSvc->start();
  BLEDevice::startAdvertising();

  delay(2000);
  Serial.println("SYSTEM RECTIFIED & ONLINE");
}

void loop() {
  if (currentMode == AUTONOMOUS)
    runAuto();
  else
    runManual();

  // Telemetry Sync
  if (millis() - lastNotify > 500) {
    char packet[20];
    snprintf(packet, sizeof(packet), "%c,%ld",
             (currentMode == MANUAL ? 'M' : 'A'), getDist());
    if (deviceConnected && pChar) {
      pChar->setValue(packet);
      pChar->notify();
    }
    lastNotify = millis();
  }
}
