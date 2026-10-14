#include <Arduino.h>
#include <Wire.h>

// Update these pins if your schematic uses different SDA/SCL pins
#define SDA_PIN 8
#define SCL_PIN 9

void setup() {
  Serial.begin(115200);
  
  // Wait up to 3 seconds for native USB CDC terminal connection
  unsigned long start = millis();
  while (!Serial && (millis() - start < 3000)) {
    delay(10);
  }

  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("\n==================================");
  Serial.println("  FitWatch I2C Hardware Scanner  ");
  Serial.println("==================================");
}

void loop() {
  Serial.println("\nScanning I2C bus...");
  byte error, address;
  int devicesFound = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("  [+] Found I2C device at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      devicesFound++;
    }
  }

  if (devicesFound == 0) {
    Serial.println("  [-] No I2C devices detected.");
  }

  Serial.println("----------------------------------");
  delay(2000); // Continuous scan every 2 seconds
}
