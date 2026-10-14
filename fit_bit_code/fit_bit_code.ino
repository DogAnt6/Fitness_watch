// =====================================================
// Code used while breadboard testing the circuit
//  ESP32 Fitness Tracker v1
//  Devices: SSD1306 OLED, GY-521 (MPU-6500), 
//           DS1307 RTC, MAX30102 Heart Rate
//  Features: NTP time sync, step counting, 
//            heart rate, watch face
// =====================================================

#include <Wire.h>
#include <WiFi.h>
#include <time.h>

// OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// RTC
#include <RTClib.h>

// IMU
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Heart rate
#include <MAX30105.h>
#include <heartRate.h>

// ─────────────────────────────────────────
//  WiFi credentials — replace these
// ─────────────────────────────────────────
const char* ssid     = "wifi_user";
const char* password = "wifi_password";

// ─────────────────────────────────────────
//  NTP settings (Toronto EDT = UTC-4)
// ─────────────────────────────────────────
const char* ntpServer      = "pool.ntp.org";
const long  gmtOffset_sec  = -14400;  // UTC-4 EDT
const int   daylightOffset = 0;       // already accounted for above

// ─────────────────────────────────────────
//  OLED (0x3C)
// ─────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ─────────────────────────────────────────
//  RTC (0x68)
// ─────────────────────────────────────────
RTC_DS3231 rtc;

// ─────────────────────────────────────────
//  IMU (0x69 — AD0 pulled HIGH)
// ─────────────────────────────────────────
Adafruit_MPU6050 mpu;

// ─────────────────────────────────────────
//  MAX30102 heart rate (0x57)
// ─────────────────────────────────────────
MAX30105 particleSensor;

// Heart rate tracking
const byte  HR_SAMPLE_SIZE = 4;           // number of samples to average BPM over
byte        hrRates[HR_SAMPLE_SIZE];      // circular buffer of recent BPM readings
byte        hrRateSpot    = 0;            // current position in buffer
long        lastBeatTime  = 0;            // time of last detected beat
float       beatsPerMinute = 0;           // current BPM
int         beatAvg       = 0;            // rolling average BPM
bool        fingerDetected = false;       // true when finger is on sensor

// Finger detection threshold — IR value below this = no finger
const long  FINGER_THRESHOLD = 50000;

// ─────────────────────────────────────────
//  Step counting
// ─────────────────────────────────────────
int           stepCount      = 0;
float         lastMagnitude  = 0;
bool          stepPending    = false;
const float   STEP_THRESHOLD = 1.2;  // g
const int     STEP_COOLDOWN  = 500;  // ms
unsigned long lastStepTime   = 0;

// ─────────────────────────────────────────
//  Button (GPIO 14, active LOW)
// ─────────────────────────────────────────
#define BUTTON_PIN 14
int currentScreen = 0;  // 0=watchface 1=steps 2=heart rate 3=accel debug

// ─────────────────────────────────────────
//  Helper — leading zero for clock
// ─────────────────────────────────────────
String padZero(int val) {
  return val < 10 ? "0" + String(val) : String(val);
}

// ─────────────────────────────────────────
//  NTP sync → DS3231
// ─────────────────────────────────────────
void syncTimeFromNTP() {
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
    display.println("Using RTC time.");
    display.display();
    delay(2000);
    WiFi.disconnect(true);
    return;
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("WiFi connected!");
  display.println("Syncing time...");
  display.display();

  configTime(gmtOffset_sec, daylightOffset, ntpServer);

  struct tm timeinfo;
  int timeout = 0;
  while (!getLocalTime(&timeinfo) && timeout < 10) {
    delay(1000);
    timeout++;
  }

  if (timeout < 10) {
    DateTime ntpTime(
      timeinfo.tm_year + 1900,
      timeinfo.tm_mon  + 1,
      timeinfo.tm_mday,
      timeinfo.tm_hour,
      timeinfo.tm_min,
      timeinfo.tm_sec
    );
    rtc.adjust(ntpTime);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Time synced!");
    display.printf("%02d:%02d:%02d", timeinfo.tm_hour,
                                     timeinfo.tm_min,
                                     timeinfo.tm_sec);
    display.display();
    delay(2000);
  } else {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("NTP timed out.");
    display.println("Using RTC time.");
    display.display();
    delay(2000);
  }

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

// ─────────────────────────────────────────
//  Resets the step count at midnight
// ─────────────────────────────────────────
void checkMidnightReset() {
  static int lastDay = -1;  // stores the last day we saw
  
  DateTime now = rtc.now();
  
  // If the day has changed since last check → reset steps
  if (lastDay == -1) {
    lastDay = now.day();  // initialize on first boot
  }
  
  if (now.day() != lastDay) {
    stepCount = 0;
    lastDay = now.day();
    Serial.println("Midnight reset — steps cleared");
  }
}

// ─────────────────────────────────────────
//  Heart rate update
//  Call this every loop — non blocking
// ─────────────────────────────────────────
void updateHeartRate() {
  long irValue = particleSensor.getIR();
  fingerDetected = irValue > FINGER_THRESHOLD;

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.printf("IR: %ld  Finger: %s\n", irValue, fingerDetected ? "YES" : "NO");
  }

  if (!fingerDetected) {
    beatAvg = 0;
    beatsPerMinute = 0;
    return;
  }

  // Rolling average to find baseline (DC component)
  static long irBuffer[32];
  static byte bufIndex = 0;
  static long irSum = 0;

  irSum -= irBuffer[bufIndex];
  irBuffer[bufIndex] = irValue;
  irSum += irValue;
  bufIndex = (bufIndex + 1) % 32;
  long irBaseline = irSum / 32;

  // AC component — how much current value deviates from baseline
  long irAC = irValue - irBaseline;

  static long lastIrAC = 0;
  static bool beatPending = false;
  static unsigned long lastBeatMS = 0;

  // Peak detection on AC signal
  // Rising edge
  if (irAC > 200 && lastIrAC <= 200) {
    beatPending = true;
  }
  // Falling edge — beat confirmed
  if (beatPending && irAC < 200) {
    unsigned long now = millis();
    long delta = now - lastBeatMS;

    if (delta > 300 && delta < 1500) {  // 40-200 BPM range
      beatsPerMinute = 60000.0 / delta;
      Serial.printf(">>> BEAT  BPM: %.1f\n", beatsPerMinute);

      hrRates[hrRateSpot++] = (byte)beatsPerMinute;
      hrRateSpot %= HR_SAMPLE_SIZE;

      beatAvg = 0;
      for (byte x = 0; x < HR_SAMPLE_SIZE; x++) beatAvg += hrRates[x];
      beatAvg /= HR_SAMPLE_SIZE;
      Serial.printf("    Avg: %d\n", beatAvg);
    }
    lastBeatMS = now;
    beatPending = false;
  }

  lastIrAC = irAC;
}

// ─────────────────────────────────────────
//  Screen 0 — Watch face
// ─────────────────────────────────────────
void drawWatchFace(DateTime now) {
  display.clearDisplay();

  // Time — 12 hour format
  int hour12 = now.hour() % 12;
  if (hour12 == 0) hour12 = 12;
  String ampm = now.hour() < 12 ? "AM" : "PM";

  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(4, 4);
  display.print(padZero(hour12));
  display.print(":");
  display.print(padZero(now.minute()));
  display.print(":");
  display.print(padZero(now.second()));

  display.setTextSize(1);
  display.setCursor(110, 6);
  display.print(ampm);

  // Date
  display.setCursor(20, 28);
  display.printf("%04d-%02d-%02d", now.year(), now.month(), now.day());

  // Divider
  display.drawLine(0, 38, 127, 38, SSD1306_WHITE);

  // Steps
  display.setCursor(0, 42);
  display.print("Steps: ");
  display.print(stepCount);

  // Heart rate
  display.setCursor(0, 54);
  display.print("HR: ");
  if (!fingerDetected) {
    display.print("-- bpm");
  } else if (beatAvg == 0) {
    display.print("... bpm");  // measuring
  } else {
    display.print(beatAvg);
    display.print(" bpm");
  }

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
//  Screen 2 — Heart rate detail
// ─────────────────────────────────────────
void drawHeartRateScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(16, 0);
  display.println("-- Heart Rate --");

  if (!fingerDetected) {
    display.setTextSize(1);
    display.setCursor(10, 25);
    display.println("Place finger on");
    display.setCursor(20, 37);
    display.println("the sensor");
  } else if (beatAvg == 0) {
    display.setTextSize(1);
    display.setCursor(20, 25);
    display.println("Measuring...");
    display.setCursor(10, 37);
    display.println("Hold still");
  } else {
    // Show BPM large
    display.setTextSize(3);
    display.setCursor(25, 18);
    display.print(beatAvg);

    display.setTextSize(1);
    display.setCursor(45, 44);
    display.println("bpm");

    // Simple heart symbol
    display.setCursor(0, 18);
    display.print("<3");
  }

  display.setCursor(0, 56);
  display.println("BTN: switch screen");
  display.display();
}

// ─────────────────────────────────────────
//  Screen 3 — Accel debug
// ─────────────────────────────────────────
void drawAccelScreen() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x / 9.81;
  float ay = a.acceleration.y / 9.81;
  float az = a.acceleration.z / 9.81;
  float mag = sqrt(ax*ax + ay*ay + az*az);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println("-- Accel Debug --");

  display.setCursor(0, 12);
  display.printf("X:   %6.3f g\n", ax);
  display.printf("Y:   %6.3f g\n", ay);
  display.printf("Z:   %6.3f g\n", az);
  display.printf("Mag: %6.3f g\n", mag);

  display.setCursor(0, 56);
  display.println("BTN: switch screen");
  display.display();
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

  if (magnitude > STEP_THRESHOLD && lastMagnitude <= STEP_THRESHOLD) {
    stepPending = true;
  }
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
//  Setup
// ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
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
  display.println("Booting...");
  display.display();
  delay(500);

  // ── DS1307 ──
  if (!rtc.begin()) {
    Serial.println("DS1307 not found!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("RTC not found!");
    display.println("Check wiring.");
    display.display();
    while (true);
  }

  // ── GY-521 (0x69) ──
  delay(250);
  mpu.begin(0x69);
  Wire.beginTransmission(0x69);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.println("GY-521 not found!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("GY-521 not found!");
    display.println("Check AD0 to 3.3V");
    display.display();
    while (true);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("GY-521 ready.");

  // ── MAX30102 (0x57) ──
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("MAX30102 not found!");
    display.println("Check wiring.");
    display.display();
    while (true);
  }
  // Configure sensor
  // LED brightness: 0=off, 255=max — 60 is good for wrist
  // Sample average: 4 samples averaged per reading
  // LED mode: 2 = Red + IR (heart rate + SpO2)
  // Sample rate: 100 samples per second
  // Pulse width: 411 microseconds
  // ADC range: 4096 nA
  particleSensor.setup(60, 4, 2, 100, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x0A);  // low red LED
  particleSensor.setPulseAmplitudeGreen(0);   // green off
  // Prefill HR buffer with realistic starting value
  for (byte i = 0; i < HR_SAMPLE_SIZE; i++) {
    hrRates[i] = 70;
  }
  Serial.println("MAX30102 ready.");

  // ── NTP → DS1307 ──
  syncTimeFromNTP();

  Serial.println("All devices ready.");
}

// ─────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────
void loop() {
  // ── Button: cycle screens ──
  static bool lastButtonState = HIGH;
  bool buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == LOW && lastButtonState == HIGH) {
    currentScreen = (currentScreen + 1) % 4;  // 4 screens now
    delay(50);
  }
  lastButtonState = buttonState;

  // ── Step detection — every iteration ──
  detectStep();

  // ── Heart rate — every iteration ──
  updateHeartRate();

  // ── Midnight reset ──
  checkMidnightReset();

  // ── Display — every 200ms ──
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay > 200) {
    lastDisplay = millis();
    DateTime now = rtc.now();
    switch (currentScreen) {
      case 0: drawWatchFace(now);    break;
      case 1: drawStepsScreen();     break;
      case 2: drawHeartRateScreen(); break;
      case 3: drawAccelScreen();     break;
    }
  }
}
