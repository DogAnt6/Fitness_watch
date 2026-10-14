// =====================================================
//  FitWatch PCB v1 Firmware
//  MCU:     ESP32-S3-WROOM-1-N16R8
//  Display: SSD1306 0.96" OLED (I2C 0x3C)
//  IMU:     MPU-6500 (I2C 0x69, AD0 HIGH)
//  HR:      MAX30102 (I2C 0x57)
//  Time:    NTP → ESP32-S3 internal RTC (no external RTC)
//  Power:   MAX30102 gated via RT9742 (GPIO4)
// =====================================================

#include <Wire.h>
#include <WiFi.h>
#include <time.h>

// OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// IMU
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Heart rate
#include <MAX30105.h>
#include "heartRate.h"

// ─────────────────────────────────────────
//  WiFi credentials
// ─────────────────────────────────────────
const char* ssid     = "BillovousUpstairs";
const char* password = "Upstairs37";

// ─────────────────────────────────────────
//  NTP settings (Toronto EDT = UTC-4)
// ─────────────────────────────────────────
const char* ntpServer     = "pool.ntp.org";
const long  gmtOffset_sec = -14400;  // UTC-4 EDT
const int   daylightOffset = 0;

// ─────────────────────────────────────────
//  Pin assignments (ESP32-S3 PCB)
// ─────────────────────────────────────────
#define SDA_PIN          8   // I2C SDA
#define SCL_PIN          9   // I2C SCL
#define BUTTON_PIN      14   // Screen cycle button (active LOW)
#define MAX30102_EN_PIN  4   // RT9742 enable (HIGH = sensor on)
#define INT_HR_PIN       5   // MAX30102 interrupt 
#define INT_IMU_PIN      6   // MPU-6500 interrupt 
#define BAT_LVL_PIN      1   // Battery voltage divider (ADC1_CH6)

// ─────────────────────────────────────────
//  OLED (0x3C)
// ─────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ─────────────────────────────────────────
//  IMU (0x69 — AD0 pulled HIGH on PCB)
// ─────────────────────────────────────────
Adafruit_MPU6050 mpu;

// ─────────────────────────────────────────
//  MAX30102 (0x57)
// ─────────────────────────────────────────
MAX30105 particleSensor;
bool hrSensorActive    = false;  // tracks whether sensor is powered

// Heart rate variables
const byte  HR_SAMPLE_SIZE  = 4;
byte        hrRates[HR_SAMPLE_SIZE];
byte        hrRateSpot      = 0;
long        lastBeatTime    = 0;
float       beatsPerMinute  = 0;
int         beatAvg         = 0;
bool        fingerDetected  = false;
const long  FINGER_THRESHOLD = 10000;
byte hrRateCount = 0;

// ─────────────────────────────────────────
//  Step counting
// ─────────────────────────────────────────
int           stepCount      = 0;
float         lastMagnitude  = 0;
bool          stepPending    = false;
const float   STEP_THRESHOLD = 1.8;   // g
const int     STEP_COOLDOWN  = 500;   // ms
unsigned long lastStepTime   = 0;

// ─────────────────────────────────────────
//  Midnight reset (using internal RTC)
// ─────────────────────────────────────────
int lastDay = -1;

// ─────────────────────────────────────────
//  NTP re-sync (every 24 hours)
// ─────────────────────────────────────────
unsigned long lastNTPSync = 0;
const unsigned long NTP_RESYNC_INTERVAL = 86400000UL; // 24 hours in ms

// ─────────────────────────────────────────
//  Battery
// ─────────────────────────────────────────
// Voltage divider: 330kΩ / 100kΩ → ratio 0.2325
const float DIVIDER_RATIO = 0.7674;
int batteryPercent = 0;

// ─────────────────────────────────────────
//  Button + screens
// ─────────────────────────────────────────
#define BUTTON_PIN      14
int currentScreen = 0;
// 0 = watch face
// 1 = steps
// 2 = heart rate
// 3 = battery + debug

// ─────────────────────────────────────────
//  Helper — leading zero
// ─────────────────────────────────────────
String padZero(int val) {
  return val < 10 ? "0" + String(val) : String(val);
}

// ─────────────────────────────────────────
//  Read battery voltage via ADC
// ─────────────────────────────────────────
float readBatteryVoltage() {
  pinMode(BAT_LVL_PIN, INPUT);
  analogSetPinAttenuation(BAT_LVL_PIN, ADC_11db);
  // Average 10 readings for stability
  int sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(BAT_LVL_PIN);
    delay(1);
  }
  float raw = sum / 10.0;
  float adcVoltage = (raw / 4095.0) * 3.3;
  float battVoltage = adcVoltage / DIVIDER_RATIO;
  return battVoltage;
}

void updateBatteryPercent() {
  float v = readBatteryVoltage();
  // 4.2V = 100%, 3.0V = 0%
  int pct = (int)((v - 3.0) / (4.2 - 3.0) * 100.0);
  batteryPercent = constrain(pct, 0, 100);
}

// ─────────────────────────────────────────
//  NTP sync → ESP32-S3 internal RTC
// ─────────────────────────────────────────
bool syncTimeFromNTP() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Connecting WiFi...");
  display.println(ssid);
  display.display();

  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi failed.");
    display.println("No time sync.");
    display.display();
    delay(2000);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return false;
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("WiFi connected!");
  display.println("Syncing time...");
  display.display();

  // Sync NTP → ESP32-S3 internal RTC
  configTime(gmtOffset_sec, daylightOffset, ntpServer);

  struct tm timeinfo;
  int timeout = 0;
  while (!getLocalTime(&timeinfo) && timeout < 10) {
    delay(1000);
    timeout++;
  }

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  if (timeout < 10) {
    lastNTPSync = millis();
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Time synced!");
    display.printf("%02d:%02d:%02d",
      timeinfo.tm_hour,
      timeinfo.tm_min,
      timeinfo.tm_sec);
    display.display();
    delay(1500);
    return true;
  } else {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("NTP timed out.");
    display.display();
    delay(1500);
    return false;
  }
}

// ─────────────────────────────────────────
//  Midnight reset using internal RTC
// ─────────────────────────────────────────
void checkMidnightReset() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;  // skip if time not set

  if (lastDay == -1) {
    lastDay = timeinfo.tm_mday;
    return;
  }

  if (timeinfo.tm_mday != lastDay) {
    stepCount  = 0;
    beatAvg    = 0;
    lastDay    = timeinfo.tm_mday;
    Serial.println("Midnight reset — steps and HR cleared");
  }
}

// ─────────────────────────────────────────
//  MAX30102 power on/off via RT9742
// ─────────────────────────────────────────
void enableHeartSensor() {
  if (hrSensorActive) return;
  digitalWrite(MAX30102_EN_PIN, HIGH);  // enable RT9742
  delay(20);  // allow power to stabilize

  if (particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    particleSensor.setup(20, 4, 2, 100, 411, 16384);
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);

    // Prefill HR buffer
    for (byte i = 0; i < HR_SAMPLE_SIZE; i++) hrRates[i] = 0;
    hrRateSpot     = 0;
    beatAvg        = 0;
    beatsPerMinute = 0;
    fingerDetected = false;

    hrSensorActive = true;
    Serial.println("MAX30102 enabled");
  } else {
    Serial.println("MAX30102 failed to init after enable");
    digitalWrite(MAX30102_EN_PIN, LOW);
  }
}

void disableHeartSensor() {
  if (!hrSensorActive) return;
  particleSensor.shutDown();
  digitalWrite(MAX30102_EN_PIN, LOW);  // cut RT9742 power
  hrSensorActive = false;
  beatAvg        = 0;
  beatsPerMinute = 0;
  fingerDetected = false;
  Serial.println("MAX30102 disabled — power cut");
}

// ─────────────────────────────────────────
//  Heart rate — custom AC/DC peak detection
// ─────────────────────────────────────────
void updateHeartRate() {
  if (!hrSensorActive) return;

  long irValue = particleSensor.getIR();

  fingerDetected = irValue > FINGER_THRESHOLD;

  // No finger → reset measurement
  if (!fingerDetected) {
    beatAvg = 0;
    beatsPerMinute = 0;
    lastBeatTime = 0;
    hrRateSpot = 0;
    hrRateCount = 0;
    return;
  }

  // Check for heartbeat
  if (checkForBeat(irValue)) {
    unsigned long now = millis();

    // Ignore first beat because there is no previous beat
    if (lastBeatTime == 0) {
      lastBeatTime = now;
      return;
    }

    long delta = now - lastBeatTime;
    lastBeatTime = now;

    if (delta > 0) {
      beatsPerMinute = 60000.0 / delta;

      // Accept reasonable heart-rate measurements
      if (beatsPerMinute > 40 && beatsPerMinute < 200) {

        // Store newest real BPM
        hrRates[hrRateSpot] = (byte)beatsPerMinute;
        hrRateSpot = (hrRateSpot + 1) % HR_SAMPLE_SIZE;

        // Count valid measurements until buffer is full
        if (hrRateCount < HR_SAMPLE_SIZE) {
          hrRateCount++;
        }

        // Average only real measurements
        int sum = 0;

        for (byte i = 0; i < hrRateCount; i++) {
          sum += hrRates[i];
        }

        beatAvg = sum / hrRateCount;
      }
    }
  }
}

// ─────────────────────────────────────────
//  Step detection
// ─────────────────────────────────────────
void detectStep() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x / 9.81;
  float ay = a.acceleration.y / 9.81;
  float az = a.acceleration.z / 9.81;
  float magnitude = sqrt(ax*ax + ay*ay + az*az);

  unsigned long now = millis();

  if (magnitude > STEP_THRESHOLD && lastMagnitude <= STEP_THRESHOLD)
    stepPending = true;

  if (stepPending && magnitude < STEP_THRESHOLD) {
    if (now - lastStepTime > STEP_COOLDOWN) {
      stepCount++;
      lastStepTime = now;
      Serial.printf("Step! Total: %d  Mag: %.3f\n", stepCount, magnitude);
    }
    stepPending = false;
  }

  lastMagnitude = magnitude;
}

// ─────────────────────────────────────────
//  Screen 0 — Watch face
// ─────────────────────────────────────────
void drawWatchFace() {
  struct tm timeinfo;
  bool timeValid = getLocalTime(&timeinfo);

  display.clearDisplay();

  // Time
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(4, 4);

  if (timeValid) {
    int hour12 = timeinfo.tm_hour % 12;
    if (hour12 == 0) hour12 = 12;
    String ampm = timeinfo.tm_hour < 12 ? "AM" : "PM";

    display.print(padZero(hour12));
    display.print(":");
    display.print(padZero(timeinfo.tm_min));
    display.print(":");
    display.print(padZero(timeinfo.tm_sec));

    display.setTextSize(1);
    display.setCursor(110, 6);
    display.print(ampm);

    // Date
    display.setCursor(20, 28);
    display.printf("%04d-%02d-%02d",
      timeinfo.tm_year + 1900,
      timeinfo.tm_mon + 1,
      timeinfo.tm_mday);
  } else {
    display.print("--:--:--");
    display.setTextSize(1);
    display.setCursor(0, 28);
    display.print("Time not set");
  }

  // Divider
  display.drawLine(0, 38, 127, 38, SSD1306_WHITE);

  // Steps
  display.setTextSize(1);
  display.setCursor(0, 42);
  display.print("Steps: ");
  display.print(stepCount);

  // Heart rate
  display.setCursor(0, 54);
  display.print("HR: ");
  if (!hrSensorActive) {
    display.print("off");
  } else if (!fingerDetected) {
    display.print("-- bpm");
  } else if (beatAvg == 0) {
    display.print("... bpm");
  } else {
    display.print(beatAvg);
    display.print(" bpm");
  }

  // Battery indicator top right
  display.setCursor(100, 54);
  display.print(batteryPercent);
  display.print("%");

  display.display();
}

// ─────────────────────────────────────────
//  Screen 1 — Step counter
// ─────────────────────────────────────────
void drawStepsScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(14, 0);
  display.println("-- Step Counter --");

  display.setTextSize(3);
  display.setCursor(20, 20);
  display.print(stepCount);

  display.setTextSize(1);
  display.setCursor(0, 56);
  display.println("BTN: switch screen");
  display.display();
}

// ─────────────────────────────────────────
//  Screen 2 — Heart rate
// ─────────────────────────────────────────
void drawHeartRateScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(16, 0);
  display.println("-- Heart Rate --");

  if (!hrSensorActive) {
    display.setCursor(20, 28);
    display.println("Sensor off");
  } else if (!fingerDetected) {
    display.setCursor(10, 25);
    display.println("Place finger on");
    display.setCursor(20, 37);
    display.println("the sensor");
  } else if (beatAvg == 0) {
    display.setCursor(20, 25);
    display.println("Measuring...");
    display.setCursor(10, 37);
    display.println("Hold still");
  } else {
    display.setTextSize(3);
    display.setCursor(25, 18);
    display.print(beatAvg);
    display.setTextSize(1);
    display.setCursor(45, 44);
    display.println("bpm");
    display.setCursor(0, 18);
    display.print("<3");
  }

  display.setCursor(0, 56);
  display.println("BTN: switch screen");
  display.display();
}

// ─────────────────────────────────────────
//  Screen 3 — Battery + system debug
// ─────────────────────────────────────────
void drawBatteryScreen() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax  = a.acceleration.x / 9.81;
  float ay  = a.acceleration.y / 9.81;
  float az  = a.acceleration.z / 9.81;
  float mag = sqrt(ax*ax + ay*ay + az*az);
  float v   = readBatteryVoltage();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println("-- System Info --");

  display.setCursor(0, 12);
  display.printf("Bat: %.2fV  %d%%\n", v, batteryPercent);
  display.printf("Mag: %.3f g\n", mag);
  display.printf("X:   %.3f g\n", ax);
  display.printf("Y:   %.3f g\n", ay);

  // Show time sync age
  unsigned long syncAge = (millis() - lastNTPSync) / 1000;
  display.setCursor(0, 52);
  if (lastNTPSync == 0) {
    display.print("NTP: never synced");
  } else {
    display.printf("NTP: %lus ago", syncAge);
  }

  display.display();
}

// ─────────────────────────────────────────
//  Periodic NTP re-sync (every 24 hours)
// ─────────────────────────────────────────
void checkNTPResync() {
  if (lastNTPSync == 0) return;  // never synced — don't try silently
  if (millis() - lastNTPSync < NTP_RESYNC_INTERVAL) return;

  Serial.println("24h NTP re-sync...");
  syncTimeFromNTP();
}

// ─────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // I2C on ESP32-S3 PCB pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // 400kHz fast mode

  // MAX30102 power gate — start OFF
  pinMode(MAX30102_EN_PIN, OUTPUT);
  digitalWrite(MAX30102_EN_PIN, LOW);

  // Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // ── OLED ──
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found!");
    while (true);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("FitWatch v1");
  display.println("Booting...");
  display.display();
  delay(500);

  // ── MPU-6500 (0x69) ──
  delay(250);
  mpu.begin(0x69);
  Wire.beginTransmission(0x69);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.println("MPU-6500 not found!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("MPU-6500 missing!");
    display.println("Check AD0 → 3.3V");
    display.display();
    while (true);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU-6500 ready at 0x69");

  // ── NTP time sync ──
  // Internal RTC only — no external RTC chip on PCB
  syncTimeFromNTP();

  // ── Initial battery reading ──
  updateBatteryPercent();

  Serial.println("FitWatch boot complete.");
}

// ─────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────
void loop() {
  // ── Button: cycle screens ──
  static bool lastButtonState = HIGH;
  bool buttonState = digitalRead(BUTTON_PIN);

  if (buttonState == LOW && lastButtonState == HIGH) {
    int prevScreen = currentScreen;
    currentScreen = (currentScreen + 1) % 4;
    delay(50);  // debounce

    // Power gate MAX30102 based on screen
    if (currentScreen == 2 || currentScreen == 0) {
      enableHeartSensor();   // screen 2 = heart rate → power on
    } else if (prevScreen == 2 || currentScreen == 0) {
      disableHeartSensor();  // leaving screen 2 → power off
    }
  }
  lastButtonState = buttonState;

  // ── Step detection — every loop ──
  detectStep();

  // ── Heart rate — only when sensor is on ──
  if (hrSensorActive) updateHeartRate();

  // ── Midnight reset ──
  checkMidnightReset();

  // ── NTP re-sync check ──
  checkNTPResync();

  // ── Battery update every 60 seconds ──
  static unsigned long lastBatUpdate = 0;
  if (millis() - lastBatUpdate > 60000) {
    lastBatUpdate = millis();
    updateBatteryPercent();
  }

  // ── Display update every 200ms (non-blocking) ──
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay > 200) {
    lastDisplay = millis();
    switch (currentScreen) {
      case 0: drawWatchFace();      break;
      case 1: drawStepsScreen();    break;
      case 2: drawHeartRateScreen(); break;
      case 3: drawBatteryScreen();  break;
    }
  }
}
