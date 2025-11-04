# ICM42688P-Voltino Arduino Library

Arduino library for the **TDK InvenSense ICM-42688-P IMU sensor**, supporting accelerometer and gyroscope with FIFO buffering.

---

## Features

- Supports both **SPI** and **I2C** communication.
- Standard mode: 16-bit data reads for accelerometer and gyroscope.  
  FIFO mode: **20-bit high-resolution data** for enhanced precision.
- FIFO packet handling for efficient data buffering and reduced CPU load.
- Configurable **Output Data Rate (ODR)** and **Full Scale Range (FSR)**.
- Data output in **g** (acceleration) and **deg/s** (gyro) units. Options for m/s² and rad/s may be added in future updates.
- Gyroscope and accelerometer **offset management**.
- **Automatic gyro calibration** at rest.
- Inspired by the [finani/ICM42688](https://github.com/finani/ICM42688) library.

---

## Installation

1. Download the library as a ZIP file from the repository.
2. In the Arduino IDE, go to **Sketch > Include Library > Add .ZIP Library** and select the downloaded file.
3. Include the library in your sketch:

#include <ICM42688P_voltino.h>
Usage
Initialize the IMU:


ICM42688P imu;
imu.begin(BUS_SPI, CS_PIN); // or BUS_I2C
Read data from FIFO:


float ax, ay, az, gx, gy, gz;
if (imu.readFIFO(ax, ay, az, gx, gy, gz)) {
    // Use the 19-bit high-resolution data
}
Set offsets:


imu.setGyroOffset(ox, oy, oz);
imu.setAccelOffset(ax, ay, az);
Get offsets:

float ox, oy, oz;
imu.getGyroOffset(ox, oy, oz);
imu.getAccelOffset(ax, ay, az);
Automatic gyro calibration:
imu.autoCalibrateGyro(1000); // Keep IMU still during calibration
Configuring Output Data Rate (ODR)
Set the Output Data Rate (ODR) for both accelerometer and gyroscope using:


imu.setODR(ICM_ODR odr);
This controls how frequently the sensor samples data. Available ODR options (from the ICM-42688-P datasheet) are:

ODR Enum	Frequency
ODR_32KHZ	32,000 Hz
ODR_16KHZ	16,000 Hz
ODR_8KHZ	8,000 Hz
ODR_4KHZ	4,000 Hz
ODR_2KHZ	2,000 Hz
ODR_1KHZ	1,000 Hz
ODR_500HZ	500 Hz (default)
ODR_200HZ	200 Hz
ODR_100HZ	100 Hz
ODR_50HZ	50 Hz
ODR_25HZ	25 Hz
ODR_12_5HZ	12.5 Hz

Note: Higher ODR values provide more frequent data but may increase power consumption and the risk of FIFO overflow if data is not read quickly enough.

Example:

imu.setODR(ODR_500HZ); // sets both accelerometer and gyro to 500 Hz

Examples
The library includes several example sketches:

SPI_FIFO_Read.ino – Initialize the IMU over SPI, set ODR to 500 Hz, optionally apply gyro offsets, read and empty FIFO, and print latest data at 40 Hz.

I2C_FIFO_Read.ino – Initialize the IMU over I2C, set ODR to 500 Hz, optionally apply gyro offsets, read and empty FIFO, and print latest data at 40 Hz.

I2C_Gyro_Calibration.ino – I2C example with automatic gyro calibration at startup to compute offsets (keep IMU still during calibration), then read and print gyro data from FIFO.

These examples are designed for Arduino-compatible boards (e.g., ESP32, Arduino Uno with SPI/I2C support). Adjust CS_PIN for SPI if needed, and ensure hardware connections are correct (SDA/SCL for I2C).

License
MIT License. See the LICENSE file for details.