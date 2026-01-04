/*
 * Example 1: Calibration Tool
 * * This script measures offsets and scaling factors.
 * Copy and paste the resulting values into the setup() function of your main program.
 *
 * * INSTRUCTIONS:
 * 'g' -> Gyro Calibration (Sensor must be perfectly still!)
 * 'a' -> Accelerometer Calibration (You will be prompted to place sensor in 6 positions)
 * 'x' -> Reset (Clears all HW offsets in the chip - first aid for weird data)
 */

#include "ICM42688P_voltino.h"

// CS Pin definition for SPI (Raspberry Pi Pico default: 17)
const uint8_t CS_PIN = 17;

// SPI Frequency Configuration
// - Raspberry Pi Pico / ESP32: 10 MHz (10000000) works great.
// - Standard Arduino (Uno/Nano/AVR): Limit to 4 MHz (4000000) to avoid errors!
const uint32_t SPI_FREQ = 10000000; 

ICM42688P IMU;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  // Initialization (configured for SPI here, change params for I2C)
  // Syntax: begin(BUS_TYPE, CS_PIN, SPI_FREQUENCY)
  if (!IMU.begin(BUS_SPI, CS_PIN, SPI_FREQ)) {
    Serial.println("IMU not found!");
    while (1);
  }
  
  Serial.println("ICM-42688-P Connected.");
  
  // Safety reset on startup (Optional)
  // Uncomment if you want to ensure chip has no internal offsets applied before starting.
  // IMU.resetHardwareOffsets();
  // Serial.println("Hardware offsets cleared (Factory reset state).");
  
  IMU.setODR(ODR_500HZ); 

  Serial.println("--------------------------------------");
  Serial.println("Ready to calibrate.");
  Serial.println("Send 'g' to calibrate GYRO.");
  Serial.println("Send 'a' to calibrate ACCELEROMETER.");
  Serial.println("Send 'x' to RESET sensor offsets.");
  Serial.println("--------------------------------------");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    
    if (c == 'g') {
      // Starts gyro calibration (1000 samples)
      // The function automatically prints the resulting code to Serial Monitor
      IMU.autoCalibrateGyro(1000);
    } 
    else if (c == 'a') {
      // Starts interactive 6-point accelerometer calibration
      // The function guides you through the process and prints the code at the end
      IMU.autoCalibrateAccel();
    }
    else if (c == 'x') {
      IMU.resetHardwareOffsets();
      Serial.println("Hardware registers cleared (Offset = 0).");
    }
  }
}