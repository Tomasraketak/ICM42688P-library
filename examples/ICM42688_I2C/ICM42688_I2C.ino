/*
 * Example 2: Basic I2C Reading with SW Calibration
 * * This script initializes the sensor, clears old (potentially incorrect) HW offsets,
 * and reads data from the FIFO buffer.
 */

#include "ICM42688P_voltino.h"

ICM42688P IMU;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Starting I2C communication...");
  
  // Default address is 0x68. CS pin is not needed for I2C.
  if (!IMU.begin(BUS_I2C)) {
    Serial.println("Sensor not found on I2C (check address 0x68)!");
    while (1);
  }

  // IMPORTANT: Clear any old HW offsets in the chip to prevent data distortion
  //IMU.resetHardwareOffsets();

  // Set sample rate
  //IMU.setODR(ODR_500HZ); 

  // --- PASTE CALIBRATION VALUES HERE (If measured) ---
  // Example:
  // IMU.setAccelOffset(0.02, -0.01, 0.05);
  // IMU.setAccelScale(1.001, 0.999, 1.002);
  // IMU.setGyroOffset(0.5, -0.2, 0.1);

  Serial.println("Sensor OK. Reading data...");
}

void loop() {
  float ax, ay, az, gx, gy, gz;

  // The readFIFO function returns true if a new complete packet is in the buffer
  // It automatically applies the SW offsets set above
  if (IMU.readFIFO(ax, ay, az, gx, gy, gz)) {
    
    // Slow down output for readability
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 50) {
      Serial.print("Accel [g]: ");
      Serial.print(ax, 3); Serial.print("\t");
      Serial.print(ay, 3); Serial.print("\t");
      Serial.print(az, 3);
      
      Serial.print("\t | Gyro [dps]: ");
      Serial.print(gx, 2); Serial.print("\t");
      Serial.print(gy, 2); Serial.print("\t");
      Serial.println(gz, 2);
      
      lastPrint = millis();
    }
  }
}