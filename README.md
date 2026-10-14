# FitWatch v1

A custom-built, feature-rich wearable smartwatch powered by an **ESP32-S3**, featuring a multi-screen UI, real-time activity tracking, on-demand optical heart rate monitoring, and Wi-Fi NTP time synchronization.

Designed from scratch in ECAD and manufactured as a fully assembled PCBA via JLCPCB.

<img width="2159" height="2371" alt="image" src="https://github.com/user-attachments/assets/787afb52-cbd9-41a6-86fb-da0f8475eed5" />

---

##  Features

* **Multi-Screen Wearable UI:** Smoothly cycle through views using a physical push-button:
  * **Watch Face:** Real-time 12-hour time, date, step count, heart rate summary, and live battery telemetry.
  * **Step Counter:** Dedicated live step tracking view.
  * **Heart Rate Monitor:** On-demand pulse oximetry and BPM readout.
  * **System Info / Debug:** Live battery voltage/percentage, MPU-6500 3-axis motion metrics, and NTP sync age.
* **Smart Activity Tracking:** Powered by an onboard 6-axis IMU with background motion filtering and step-detection algorithms.
* **On-Demand Health Monitoring:** Features a MAX30102 optical sensor for pulse detection, automatically power-gated when navigating away from the heart rate screen to conserve power.
* **Wi-Fi & NTP Time Sync:** Automatically connects to Wi-Fi on boot to fetch precise network time, backing it up to the ESP32-S3's internal RTC for accurate offline timekeeping.
* **Dynamic Power Management:** Complete hardware power-gating of the 1.8V sensor rail via an RT9742 load switch and precision analog voltage telemetry.

---

##  Hardware & Bill of Materials

* **Microcontroller:** ESP32-S3-WROOM-1-N16R8 (Dual-core Xtensa, Wi-Fi / BLE, Native USB CDC/JTAG)
* **Display:** 0.96" SSD1306 OLED (`0x3C`) communicating over Hardware I2C
* **IMU:** MPU-6500 (`0x69`, AD0 pulled HIGH) for motion and step tracking
* **Heart Rate / SpO2:** MAX30102 (`0x57`) power-gated via RT9742 load switch (`GPIO4`)
* **Power Regulation:**
  * AP2112K (3.3V Main Rail)
  * TLV70018 (1.8V Sensor Rail)
* **Telemetry:** Calibrated resistor voltage divider (330k Omega / 100k Omega, ratio 0.7674) tied to ADC1 (`GPIO7`)

Schematics, PCB layout, and manufacturing files (Gerbers, BOM, pick-and-place) are in the [`hardware/`](hardware/) folder.

---

##  Pinout Mapping

| Function | Pin (ESP32-S3) | Description |
| :--- | :--- | :--- |
| **SDA** | GPIO 8 | I2C Data Line |
| **SCL** | GPIO 9 | I2C Clock Line |
| **MAX_EN** | GPIO 4 | RT9742 Load Switch Enable (1.8V rail) |
| **Button** | GPIO 14 | Screen Cycle Button (Active LOW with Pull-Up) |
| **Bat Sense** | GPIO 7 | Battery Voltage Divider (ADC1 Channel 6) |
| **IMU INT** | GPIO 6 | MPU-6500 Interrupt (Optional/Unused v1) |
| **HR INT** | GPIO 5 | MAX30102 Interrupt (Optional/Unused v1) |

### I²C Addresses

| Device | Address |
| :--- | :--- |
| SSD1306 OLED | `0x3C` |
| MPU-6500 IMU | `0x69` |
| MAX30102 | `0x57` |

---

## 💻 Firmware Architecture

Built using the Arduino framework for ESP32, the code is structured around non-blocking timing intervals and state-machine screen rendering:
1. **`setup()`**: Initializes native USB CDC, configures I2C at 400kHz, checks peripheral health (`0x3C`, `0x69`), syncs time via NTP, and reads initial battery levels.
2. **`loop()`**: Handles non-blocking button debouncing, background step detection (`detectStep`), conditional heart rate calculation when the sensor rail is active, and periodic display refreshes.

### Dependencies
Make sure you have the following libraries installed in your Arduino IDE:
* `Adafruit_GFX`
* `Adafruit_SSD1306`
* `Adafruit_MPU6050`
* `MAX30105` (SparkFun MAX30xX Library)

---

<img width="2159" height="2371" alt="image" src="https://github.com/user-attachments/assets/7f921155-400e-4d20-8454-f5ceadc93dee" />

## 📄 License

Distributed under the MIT License. See [`LICENSE`](LICENSE) for details.
