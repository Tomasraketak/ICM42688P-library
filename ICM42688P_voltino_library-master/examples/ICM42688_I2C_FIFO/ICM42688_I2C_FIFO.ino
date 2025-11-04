/*
 * ICM42688P-Voltino Library Example: I2C FIFO Read
 * 
 * This example demonstrates how to initialize the ICM-42688-P IMU sensor over I2C,
 * set the Output Data Rate (ODR) to 500 Hz for both accelerometer and gyroscope,
 * and continuously read data from the FIFO buffer.
 * 
 * The FIFO is emptied in a loop to prevent overflow, and the latest readings are
 * printed to the Serial Monitor at approximately 40 Hz (every 25 ms) for readability.
 * Data is output in g (acceleration) and dps (degrees per second, gyro).
 * 
 * Hardware Requirements:
 * - Connect the IMU to your Arduino board via I2C:
 *   - SDA to SDA pin, SCL to SCL pin.
 *   - Address is 0x68 (default).
 * - No CS pin is needed for I2C.
 * 
 * Usage:
 * 1. Upload this sketch to your board.
 * 2. Open the Serial Monitor at 115200 baud.
 * 3. Observe the accel and gyro data being printed.
 * 
 * Note: If using a different ODR, adjust imu.setODR() accordingly. I2C may be slower than SPI for high ODR, so monitor for potential FIFO overflows.
 * 
 * Calibration: You can manually set gyro offsets obtained from the I2C_Gyro_Calibration example.
 * Run that example first, copy the printed offsets, and paste them into setGyroOffset() below.
 */

#include <ICM42688P_voltino.h>

ICM42688P imu;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Initializing IMU (I2C)...");
  if (!imu.begin(BUS_I2C)) {  // No CS pin needed for I2C
    Serial.println("ERROR: IMU not detected!");
    while (1);
  }
  Serial.println("IMU initialized.");
  imu.setODR(ODR_500HZ); // 500 Hz sampling

  // Set calibrated gyro offsets (replace with values from I2C_Gyro_Calibration example, e.g., 0.00123, -0.00456, 0.00789)
  imu.setGyroOffset(0.0, 0.0, 0.0); // Placeholder values; update with your measured offsets
}

void loop() {
  static uint32_t lastPrint = 0;
  static float lastAx = 0, lastAy = 0, lastAz = 0, lastGx = 0, lastGy = 0, lastGz = 0;
  bool anyRead = false;
  // --- Empty FIFO: read until no more complete packets ---
  while (true) {
    float ax, ay, az, gx, gy, gz;
    if (!imu.readFIFO(ax, ay, az, gx, gy, gz)) break;
    lastAx = ax; lastAy = ay; lastAz = az;
    lastGx = gx; lastGy = gy; lastGz = gz;
    anyRead = true;
    yield();
  }
  // --- Print only at 40 Hz (every 25 ms) and only if we have data ---
  if ((millis() - lastPrint) >= 25 && anyRead) {
    lastPrint = millis();
    Serial.print("FIFO Accel (g): ");
    Serial.print(lastAx, 6); Serial.print(", ");
    Serial.print(lastAy, 6); Serial.print(", ");
    Serial.print(lastAz, 6);
    Serial.print(" Gyro (dps): ");
    Serial.print(lastGx, 6); Serial.print(", ");
    Serial.print(lastGy, 6); Serial.print(", ");
    Serial.println(lastGz, 6);
  }
}