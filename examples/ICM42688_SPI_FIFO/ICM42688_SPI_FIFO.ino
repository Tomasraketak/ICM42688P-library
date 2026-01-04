/*
 * Example 3: SPI Communication with FIFO, ODR, and SW Calibration
 * * This example uses the "Safe" version of the library with Software Offsets.
 * Offsets are subtracted in the processor, not in the sensor chip.
 * * SPI SPEED NOTE:
 * You can now define the SPI frequency in the begin() function.
 * - Raspberry Pi Pico / ESP32: Can handle 10 MHz (10000000) easily.
 * - Arduino Uno / Nano (AVR): MAX 4 MHz (4000000) is recommended. 
 * Higher speeds on AVR may cause data corruption or freeze.
 */

#include "ICM42688P_voltino.h"

// Define CS Pin (Default GP17 for Raspberry Pi Pico)
const uint8_t CS_PIN = 17;

// SPI Frequency Configuration
// Change this to 4000000 if using standard Arduino (Uno/Nano)
const uint32_t SPI_FREQ = 10000000; 

ICM42688P IMU;

void setup() {
  Serial.begin(115200);
  // while (!Serial); // Uncomment to wait for USB Serial

  // 1. Initialization via SPI
  // Syntax: begin(BUS_TYPE, CS_PIN, SPI_FREQUENCY)
  if (!IMU.begin(BUS_SPI, CS_PIN, SPI_FREQ)) {
    Serial.println("SPI Communication Error! Check wiring.");
    while (1);
  }
  Serial.print("ICM-42688-P successfully connected via SPI at ");
  Serial.print(SPI_FREQ / 1000000);
  Serial.println(" MHz.");

  // IMPORTANT: Clear any old/bad hardware calibrations directly in the chip
  // to ensure we start with a clean slate for SW calibration.
  // IMU.resetHardwareOffsets();

  // 2. Set Output Data Rate (ODR)
  IMU.setODR(ODR_4KHZ);

  // =============================================================
  // 3. CALIBRATION SEQUENCE (SW OFFSETS)
  // =============================================================
  
  // A) Accelerometer (Bias + Scale)
  // Using setAccelOffset (Software) instead of HardwareOffset.
  // Insert values obtained from the Calibration Tool here.
  IMU.setAccelOffset(-0.0019, 0.0012, -0.0010); 
  IMU.setAccelScale(1.0061, 1.0033, 0.9996);

  // B) Gyroscope (Bias)
  // Option 1: Automatic calibration on every startup (Recommended for gyro)
  Serial.println("Calibrating Gyro (Please keep still)...");
  IMU.autoCalibrateGyro(500); // Measures bias and stores it in SW variables
  
  // Option 2: Manual setting (if values are known and you want fast boot)
  // If you uncomment this, comment out autoCalibrateGyro above.
  // IMU.setGyroOffset(0.61, -0.27, 0.16); 

  Serial.println("Setup complete. Starting loop.");
}

void loop() {
  float ax, ay, az, gx, gy, gz;

  // Read FIFO
  // readFIFO automatically subtracts the configured SW offsets
  while (IMU.readFIFO(ax, ay, az, gx, gy, gz)) {
    
    // Data Output
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 50) {
      Serial.print("A [g]: ");
      Serial.print(ax, 3); Serial.print(", ");
      Serial.print(ay, 3); Serial.print(", ");
      Serial.print(az, 3);
      
      Serial.print(" | G [dps]: ");
      Serial.print(gx, 2); Serial.print(", ");
      Serial.print(gy, 2); Serial.print(", ");
      Serial.println(gz, 2);
      
      lastPrint = millis();
    }
  }
}