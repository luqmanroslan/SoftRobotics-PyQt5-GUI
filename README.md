# Soft Robot Control System (GUI & Software Framework)

This repository contains the **Python-based software framework** for controlling a soft robot.
It features a graphical user interface (GUI), real-time hardware communication, and integration with predictive kinematic models.

---

## 🚀 Project Overview

* Built with **PyQt5** for real-time control and visualization.
* Communicates with an **ESP32 microcontroller** via serial.
* Multiple control modes:

  * Joystick control
  * Automated dance sequences
  * Direct actuator commands
* Real-time sensor monitoring (IMU, potentiometer, etc.).
* Designed to integrate with **machine learning models** for forward & inverse kinematics.

---

## 📂 Directory Structure

```
.
├── ml_main.py                # Main entry point for GUI
├── communication/            # Serial communication modules (ESP32 interface)
├── control/                  # High-level control & ML processor
├── gui/                      # All GUI components and custom widgets
└── requirements.txt          # Python dependencies
```

---

## 🔗 Related Repositories

To keep this repo modular and lightweight, supporting components are maintained separately:

* [**SoftRobotics-Firmware**](https://github.com/luqmanroslan/SoftRobotics-Firmware) – ESP32 firmware for actuator and sensor control.
* [**SoftRobotics-MLmodels**](https://github.com/luqmanroslan/SoftRobotics-MLmodels) – Training scripts and pre-trained predictive kinematic models.

---

## ⚙️ Setup Instructions

1. **Clone this repository**

   ```bash
   git clone https://github.com/luqmanroslan/SoftRobotics-PyQt5-GUI-Clean.git
   cd SoftRobotics-PyQt5-GUI-Clean
   ```

2. **Install dependencies** (recommended: use a virtual environment)

   ```bash
   pip install -r requirements.txt
   ```

3. **Run the application**

   ```bash
   python ml_main.py
   ```

---

## ✨ Key Features

* ✅ Clean, modular **PyQt5 GUI**
* ✅ Robust **serial communication** with ESP32
* ✅ Real-time **sensor visualization**
* ✅ Extendable with **ML-based predictive kinematics**

---

## 📝 Notes

* This repo only contains the **software & GUI framework**.
* Firmware and ML training resources are maintained in the linked repositories above.
* Some configuration (e.g., COM port, hardware pin mapping) may need to be adapted for your setup.

---

📌 Maintained by [**Luqman Hakeem Roslan**](https://github.com/luqmanroslan)

