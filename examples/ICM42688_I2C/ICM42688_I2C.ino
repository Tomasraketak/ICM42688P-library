/*
 * Example 1: Basic I2C Reading with SW Calibration
 * Uses the new simplified beginI2C() and universal readIMU() methods.
 * By default, this uses FIFO_NONE (direct register polling).
 */

#include "ICM42688P_voltino.h"

ICM42688P IMU;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Starting I2C communication...");

  // Enable debug warnings (e.g. for Bandwidth limits)
  IMU.setDebug(true);
  
  // Initialize I2C. Default is 400kHz, Address 0x68.
  // For ESP32 custom pins, use: IMU.beginI2C(400000, 0x68, SDA_PIN, SCL_PIN);
  if (!IMU.beginI2C()) {
    Serial.println("Sensor not found on I2C (check wiring or address)!");
    while (1);
  }

  // Set sample rate (ODR)
  IMU.setODR(ODR_500HZ); 

  // --- PASTE CALIBRATION VALUES HERE (If measured) ---
  // IMU.setAccelOffset(0.02, -0.01, 0.05);
  // IMU.setAccelScale(1.001, 0.999, 1.002);
  // IMU.setGyroOffset(0.5, -0.2, 0.1);

  Serial.println("Sensor OK. Reading data...");
}

void loop() {
  float ax, ay, az, gx, gy, gz;

  // readIMU automatically knows we are not using FIFO (default state)
  if (IMU.readIMU(ax, ay, az, gx, gy, gz)) {
    Serial.print("Accel [g]: ");
    Serial.print(ax, 3); Serial.print(", ");
    Serial.print(ay, 3); Serial.print(", ");
    Serial.print(az, 3);
    
    Serial.print(" | Gyro [dps]: ");
    Serial.print(gx, 2); Serial.print(", ");
    Serial.print(gy, 2); Serial.print(", ");
    Serial.println(gz, 2);
  }
  
  delay(10); // Standard delay for basic polling
}