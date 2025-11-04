/*
 * ICM42688P-Voltino Library Example: I2C Gyro Calibration
 * 
 * This example demonstrates how to initialize the ICM-42688-P IMU sensor over I2C,
 * perform automatic gyro calibration at startup with 5000 samples,
 * retrieve and print the stored gyro offsets,
 * and then read gyro data from the FIFO buffer in loop.
 * 
 * The calibration function (autoCalibrateGyro) averages samples while the IMU is kept still
 * to compute and set gyro offsets, reducing drift in readings.
 * 
 * After calibration, it reads FIFO data if available and prints only gyro values.
 * 
 * Hardware Requirements:
 * - Connect the IMU to your Arduino board via I2C:
 *   - SDA to SDA pin, SCL to SCL pin.
 *   - Address is 0x68 (default).
 * - No CS pin is needed for I2C, but the code includes pinMode for pin 17 (optional, can be removed if not used).
 * 
 * Usage:
 * 1. Place the IMU on a flat, stable surface before running.
 * 2. Upload this sketch.
 * 3. Open the Serial Monitor at 115200 baud.
 * 4. Wait for the "Starting gyro auto-calibration... Keep IMU still." message, and do not move the device during calibration.
 * 5. Note the stored offsets printed after calibration – use them in other examples via setGyroOffset().
 * 6. Observe the gyro data in loop.
 * 
 * Note: Calibration is done only once in setup(). For recalibration, restart the sketch.
 * This example focuses on gyro; accel data is read but not printed.
 */

#include <ICM42688P_voltino.h>

ICM42688P imu;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Starting gyro auto-calibration...");
  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);

  if (!imu.begin(BUS_I2C)) {
    Serial.println("IMU not found!");
    while (1);
  }

  // Automatic gyro calibration at rest (approx. 3s with 5000 samples)
  imu.autoCalibrateGyro(5000);

  // Retrieve the resulting offsets
  float ox, oy, oz;
  imu.getGyroOffset(ox, oy, oz);

  Serial.print("Stored offsets: ");
  Serial.print(ox, 5); Serial.print(", ");
  Serial.print(oy, 5); Serial.print(", ");
  Serial.println(oz, 5);
  delay(5000);
}

void loop() {
  float ax, ay, az, gx, gy, gz;
  if (imu.readFIFO(ax, ay, az, gx, gy, gz)) {
    Serial.print("Gyro (°/s): ");
    Serial.print(gx, 3); Serial.print(", ");
    Serial.print(gy, 3); Serial.print(", ");
    Serial.println(gz, 3);
  }
}