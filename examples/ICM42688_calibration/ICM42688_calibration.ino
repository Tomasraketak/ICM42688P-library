/*
 * Example 3: Calibration Tool
 * Measures your specific chip's hardware biases and scaling errors.
 * Copy and paste the resulting values into the setup() function of your main code.
 *
 * INSTRUCTIONS:
 * Send 'g' via Serial Monitor -> Gyro Calibration (Sensor must be perfectly still)
 * Send 'a' via Serial Monitor -> Accelerometer 6-Point Calibration
 */

#include "ICM42688P_voltino.h"

const uint8_t CS_PIN = 17;
const uint32_t SPI_FREQ = 10000000; 

ICM42688P IMU;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  // Initialization - If you are using I2C, replace this with IMU.beginI2C();
  if (!IMU.beginSPI(CS_PIN, SPI_FREQ)) {
    Serial.println("IMU not found!");
    while (1);
  }
  
  Serial.println("ICM-42688-P Connected.");
  
  // Turn off FIFO so calibration happens in real-time synchronous mode
  IMU.setFIFOMode(FIFO_NONE);
  IMU.setODR(ODR_500HZ); 

  Serial.println("--------------------------------------");
  Serial.println("Ready to calibrate.");
  Serial.println("Send 'g' to calibrate GYRO.");
  Serial.println("Send 'a' to calibrate ACCELEROMETER.");
  Serial.println("Send 'x' to RESET sensor offsets in RAM.");
  Serial.println("--------------------------------------");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    
    if (c == 'g') {
      IMU.autoCalibrateGyro(1000); // 1000 samples for gyro
    } 
    else if (c == 'a') {
      IMU.autoCalibrateAccel();    // Interactive 6-point accel calibration
    }
    else if (c == 'x') {
      IMU.setGyroOffset(0,0,0);
      IMU.setAccelOffset(0,0,0);
      IMU.setAccelScale(1,1,1);
      Serial.println("All offsets reset to zero.");
    }
  }
}