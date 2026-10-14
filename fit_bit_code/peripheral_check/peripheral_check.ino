#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

#define SDA_PIN 8
#define SCL_PIN 9
#define MAX_EN_PIN 4  // Set to your schematic's MAX30102 / 1.8V LDO EN GPIO

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200);

  // Enable the 1.8V rail for MAX30102
  pinMode(MAX_EN_PIN, OUTPUT);
  digitalWrite(MAX_EN_PIN, HIGH);
  delay(50); 

  Wire.begin(SDA_PIN, SCL_PIN);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed!");
    for(;;);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("FitWatch Diagnostics");
  display.println("---------------------");

  // Check for MAX30102 (0x57) and IMU (0x69)
  bool maxFound = false, imuFound = false;
  
  Wire.beginTransmission(0x57);
  if (Wire.endTransmission() == 0) maxFound = true;

  Wire.beginTransmission(0x69);
  if (Wire.endTransmission() == 0) imuFound = true;

  display.print("OLED (0x3C):  "); display.println("OK");
  display.print("IMU  (0x69):  "); display.println(imuFound ? "OK" : "MISSING");
  display.print("heart  (0x57):  "); display.println(maxFound ? "OK" : "MISSING");
  
  display.display();
}

void loop() {}
