#include "ICM42688P_voltino.h"

ICM42688P IMU;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Inicializace (I2C)
  if (!IMU.begin(BUS_I2C)) {
    Serial.println("ICM42688P not found!");
    while (1);
  }
  
  Serial.println("ICM42688P Found!");
  Serial.println("Send 'g' to calibrate GYRO (keep still).");
  Serial.println("Send 'a' to calibrate ACCEL (6 positions).");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    
    if (c == 'g') {
      IMU.autoCalibrateGyro(300); // Automaticky zapíše do HW registrů
    } 
    else if (c == 'a') {
      IMU.autoCalibrateAccel(); // Spustí průvodce 6-bodovou kalibrací
    }
  }
}