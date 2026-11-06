// =====================================================
//  FitWatch PCB v1 Firmware
//  MCU:     ESP32-S3-WROOM-1-N16R8
//  Display: SSD1306 0.96" OLED (I2C 0x3C)
//  IMU:     MPU-6500 (I2C 0x69, AD0 HIGH)
//  HR:      MAX30102 (I2C 0x57)
//  Time:    NTP → ESP32-S3 internal RTC (no external RTC)
//  Power:   MAX30102 gated via RT9742 (GPIO4)
//
//  v1.1: WiFi/NTP sync is now NON-BLOCKING (state machine).
// =====================================================

#include <Wire.h>
#include <WiFi.h>
#include <time.h>
#include "esp_sntp.h"

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
const char* ssid     = "Anthony ";
const char* password = "Pineapple";

// ─────────────────────────────────────────
//  NTP settings (Toronto EDT = UTC-4)
// ─────────────────────────────────────────
const char* ntpServer     = "pool.ntp.org";
const long  gmtOffset_sec = -18000;  // UTC-4 EDT
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
//  Non-blocking time sync (WiFi + NTP)
// ─────────────────────────────────────────
enum SyncState { SYNC_IDLE, SYNC_WIFI_CONNECTING, SYNC_NTP_WAITING };
SyncState syncState = SYNC_IDLE;

unsigned long syncStateStart = 0;   // when current state began
unsigned long lastNTPSync    = 0;   // millis() of last success (0 = never)
unsigned long nextSyncAt     = 0;   // millis() when next attempt is allowed

const unsigned long WIFI_TIMEOUT_MS     = 15000UL;     // give up on WiFi
const unsigned long NTP_TIMEOUT_MS      = 10000UL;     // give up on NTP
const unsigned long NTP_RESYNC_INTERVAL = 86400000UL;  // 24 h after success
const unsigned long NTP_RETRY_INTERVAL  = 300000UL;    // 5 min after failure

volatile bool ntpSyncedFlag = false;  // set by SNTP callback

void onTimeSynced(struct timeval* tv) {
  ntpSyncedFlag = true;
}

// ─────────────────────────────────────────
//  Battery
// ─────────────────────────────────────────
const float DIVIDER_RATIO = 0.7674;
int batteryPercent = 0;

// ─────────────────────────────────────────
//  Button + screens
// ─────────────────────────────────────────
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
//  Helper — read RTC WITHOUT blocking.
//  getLocalTime() defaults to a 5 second wait when the
//  time isn't set, which froze the whole loop offline.
// ─────────────────────────────────────────
bool getTimeNow(struct tm* info) {
  return getLocalTime(info, 0);
}

// ─────────────────────────────────────────
//  Read battery voltage via ADC
// ─────────────────────────────────────────
float readBatteryVoltage() {
  pinMode(BAT_LVL_PIN, INPUT);
  analogSetPinAttenuation(BAT_LVL_PIN, ADC_11db);
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
//  Non-blocking NTP sync → internal RTC
// ─────────────────────────────────────────
void startTimeSync() {
  if (syncState != SYNC_IDLE) return;
  Serial.println("Time sync: starting WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);   // returns immediately
  syncState      = SYNC_WIFI_CONNECTING;
  syncStateStart = millis();
}

void finishTimeSync(bool success) {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  syncState = SYNC_IDLE;

  if (success) {
    lastNTPSync = millis();
    if (lastNTPSync == 0) lastNTPSync = 1;  // 0 is reserved for "never"
    nextSyncAt  = millis() + NTP_RESYNC_INTERVAL;
    Serial.println("Time sync: success");
  } else {
    nextSyncAt  = millis() + NTP_RETRY_INTERVAL;
    Serial.println("Time sync: failed, will retry later");
  }
}

// Call every loop() — does a tiny bit of work and returns
void updateTimeSync() {
  switch (syncState) {

    case SYNC_IDLE:
      // Start a new attempt when it's due (retries/resyncs happen automatically)
      if ((long)(millis() - nextSyncAt) >= 0) startTimeSync();
      break;

    case SYNC_WIFI_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Time sync: WiFi up, requesting NTP...");
        ntpSyncedFlag = false;
        sntp_set_time_sync_notification_cb(onTimeSynced);
        configTime(gmtOffset_sec, daylightOffset, ntpServer);
        syncState      = SYNC_NTP_WAITING;
        syncStateStart = millis();
      } else if (millis() - syncStateStart > WIFI_TIMEOUT_MS) {
        Serial.println("Time sync: WiFi timeout");
        finishTimeSync(false);
      }
      break;

    case SYNC_NTP_WAITING:
      if (ntpSyncedFlag) {
        finishTimeSync(true);
      } else if (millis() - syncStateStart > NTP_TIMEOUT_MS) {
        Serial.println("Time sync: NTP timeout");
        finishTimeSync(false);
      }
      break;
  }
}

// ─────────────────────────────────────────
//  Midnight reset using internal RTC
// ─────────────────────────────────────────
void checkMidnightReset() {
  struct tm timeinfo;
  if (!getTimeNow(&timeinfo)) return;  // skip if time not set

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

  if (!fingerDetected) {
    beatAvg = 0;
    beatsPerMinute = 0;
    lastBeatTime = 0;
    hrRateSpot = 0;
    hrRateCount = 0;
    return;
  }

  if (checkForBeat(irValue)) {
    unsigned long now = millis();

    if (lastBeatTime == 0) {
      lastBeatTime = now;
      return;
    }

    long delta = now - lastBeatTime;
    lastBeatTime = now;

    if (delta > 0) {
      beatsPerMinute = 60000.0 / delta;

      if (beatsPerMinute > 40 && beatsPerMinute < 200) {
        hrRates[hrRateSpot] = (byte)beatsPerMinute;
        hrRateSpot = (hrRateSpot + 1) % HR_SAMPLE_SIZE;

        if (hrRateCount < HR_SAMPLE_SIZE) {
          hrRateCount++;
        }

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
  bool timeValid = getTimeNow(&timeinfo);

  display.clearDisplay();

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

    display.setCursor(20, 28);
    display.printf("%04d-%02d-%02d",
      timeinfo.tm_year + 1900,
      timeinfo.tm_mon + 1,
      timeinfo.tm_mday);
  } else {
    display.print("--:--:--");
    display.setTextSize(1);
    display.setCursor(0, 28);
    display.print(syncState != SYNC_IDLE ? "Syncing time..." : "Time not set");
  }

  display.drawLine(0, 38, 127, 38, SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 42);
  display.print("Steps: ");
  display.print(stepCount);

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

  // Small "*" while a sync is in progress
  if (syncState != SYNC_IDLE) {
    display.setCursor(88, 54);
    display.print("*");
  }

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

  display.setCursor(0, 52);
  if (syncState == SYNC_WIFI_CONNECTING) {
    display.print("NTP: WiFi...");
  } else if (syncState == SYNC_NTP_WAITING) {
    display.print("NTP: syncing...");
  } else if (lastNTPSync == 0) {
    display.print("NTP: never synced");
  } else {
    display.printf("NTP: %lus ago", (millis() - lastNTPSync) / 1000);
  }

  display.display();
}

// ─────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  pinMode(MAX30102_EN_PIN, OUTPUT);
  digitalWrite(MAX30102_EN_PIN, LOW);

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

  // ── Time sync: kick it off, but do NOT wait for it ──
  startTimeSync();

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

    if (currentScreen == 2 || currentScreen == 0) {
      enableHeartSensor();
    } else if (prevScreen == 2 || currentScreen == 0) {
      disableHeartSensor();
    }
  }
  lastButtonState = buttonState;

  // ── Time sync state machine (never blocks) ──
  updateTimeSync();

  // ── Step detection — every loop ──
  detectStep();

  // ── Heart rate — only when sensor is on ──
  if (hrSensorActive) updateHeartRate();

  // ── Midnight reset ──
  checkMidnightReset();

  // ── Battery update every 60 seconds ──
  static unsigned long lastBatUpdate = 0;
  if (millis() - lastBatUpdate > 60000) {
    lastBatUpdate = millis();
    updateBatteryPercent();
  }

  // ── Display update every 200ms ──
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay > 200) {
    lastDisplay = millis();
    switch (currentScreen) {
      case 0: drawWatchFace();       break;
      case 1: drawStepsScreen();     break;
      case 2: drawHeartRateScreen(); break;
      case 3: drawBatteryScreen();   break;
    }
  }
}
