# 🤖 ROBOROVER PRO - Autonomous AGV

**ROBOROVER** is a semi-autonomous Automated Guided Vehicle (AGV) powered by the **ESP32**. It features a dual-mode operation system: high-precision manual steering via a web-based BLE joystick dashboard and fully autonomous PID-based line following with obstacle avoidance.

---
DASHBOARD LINK: https://DimpalMahitkar.github.io/ROBOROVER/

## 🚀 Key Features

- 🎮 **Pro Cockpit Dashboard**: A futuristic, web-based BLE interface with a virtual flight stick (Joystick).
- 🧠 **AI Autonomous Mode**: Advanced PID control algorithm for smooth black-line tracking.
- 🚧 **S-Curve Avoidance**: Intelligent obstacle navigation. Scans surroundings with a servo and dynamically calculates a "Box" bypass maneuver.
- ⚡ **Torque Optimized**: Designed to carry 400g–500g payloads on inclines of 5°–10°.
- 📡 **BLE Connectivity**: Low-energy, high-bandwidth communication directly from a web browser.

---

## 🛠 Hardware Configuration

### Core Components
- **Microcontroller**: ESP32-WROOM-32
- **Motor Driver**: L298N Dual H-Bridge
- **Power**: 7.4V - 12V Li-ion Battery with MP1584EN Buck Converter (Regulated 5V)
- **Sensors**: 
  - Ultrasonic HC-SR04 (Distance)
  - QTR-5 IR Array (Line Sensing)
  - SG90 Servo (Radar Scanning)

### 📍 Pin Mapping
| Component | ESP32 Pin |
| :--- | :--- |
| **Motor L (ENA, IN1, IN2)** | 13, 14, 27 |
| **Motor R (ENB, IN3, IN4)** | 12, 26, 25 |
| **IR S2, S3, S4** | 33, 34, 35 |
| **Ultrasonic TRIG** | 5 |
| **Ultrasonic ECHO** | 18 |
| **Servo PWM** | 19 |

---

## 📥 Installation

### 1. Arduino Setup
1. Install the [ESP32 Board Package](https://github.com/espressif/arduino-esp32).
2. Install the **ESP32Servo** library via the Library Manager.
3. Upload `ROBOROVER.ino` to your ESP32.

### 2. Dashboard Deployment
1. Upload the `index.html` file to your GitHub repository.
2. Go to **Settings > Pages**.
3. Set the branch to **`main`** and click **Save**.
4. Open the generated HTTPS link on your mobile browser (Chrome recommended).

---

## 🕹 Operation Guide

### Manual Mode
- **Initial Link**: Tap the connection bar in the dashboard to pair with `ROBOROVER_BLE`.
- **Joystick**: Drag the center knob to drive. Release to snap back and stop.
- **HUD**: Monitor real-time distance and mode status in the header.

### Autonomous Mode
- Place the robot on a black line.
- Select **AI AUTONOMOUS** on the dashboard.
- The robot will track the line at `baseSpeed (100)`. 
- If an object is detected within **20cm**, the robot will scan, bypass, and realign itself with the line automatically.

---

## 📜 License
This project is open-source. Feel free to fork and improve! 🛠️
