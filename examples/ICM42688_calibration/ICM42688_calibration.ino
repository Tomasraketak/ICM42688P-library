#include "ICM42688P_voltino.h"

// CS pin 17 pro Pico (SPI)
const uint8_t CS_PIN = 17;
ICM42688P IMU;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  // Inicializace SPI
  if (!IMU.begin(BUS_SPI, CS_PIN)) {
    Serial.println("IMU not found!");
    while (1);
  }
  
  Serial.println("ICM-42688-P Connected.");
  
  // DŮLEŽITÉ: Při startu tohoto skriptu resetujeme offsets,
  // abychom se zbavili případného zaseknutého stavu (-2001 dps).
  IMU.resetHardwareOffsets();
  Serial.println("Hardware offsets reset to 0.");

  Serial.println("Commands:");
  Serial.println(" 'g' - Calibrate Gyro (Keep Still)");
  Serial.println(" 'a' - Calibrate Accel (6-Point)");
  Serial.println(" 'x' - RESET ALL OFFSETS (Emergency Fix)");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    
    if (c == 'g') {
      IMU.autoCalibrateGyro(1000);
    } 
    else if (c == 'a') {
      IMU.autoCalibrateAccel();
    }
    else if (c == 'x') {
      IMU.resetHardwareOffsets();
      Serial.println("Offsets cleared.");
    }
  }
}