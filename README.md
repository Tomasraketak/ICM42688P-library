# ICM42688P-Voltino Arduino Library

Arduino library for the **TDK InvenSense ICM-42688-P IMU sensor**, supporting accelerometer and gyroscope with FIFO buffering.

---

## Features

- Supports both **SPI** and **I2C** communication.
- Standard mode: 16-bit data reads for accelerometer and gyroscope.  
  FIFO mode: **20-bit high-resolution data** for enhanced precision.
- FIFO packet handling for efficient data buffering and reduced CPU load.
- Configurable **Output Data Rate (ODR)** and **Full Scale Range (FSR)**.
- Data output in **g** (acceleration) and **deg/s** (gyro).
- **Advanced Calibration**:
  - **Hardware Offsets**: Writes bias directly to sensor registers (sensor subtracts bias internally).
  - **Software Offsets**: Applied by the library on the MCU side.
  - **Accelerometer Software Scaling**: Correction for scaling errors.
- **Automatic Calibration Tools**:
  - Automatic gyro bias calibration (at rest).
  - Interactive 6-point accelerometer calibration (computes hardware bias + software scale).
- Inspired by the [finani/ICM42688](https://github.com/finani/ICM42688) library.

---

## Installation

1. Download the library as a ZIP file from the repository.
2. In the Arduino IDE, go to **Sketch > Include Library > Add .ZIP Library** and select the downloaded file.
3. Include the library in your sketch:

```cpp
#include <ICM42688P_voltino.h>
```

---

## Usage

### 1. Initialization

Initialize the IMU with I2C or SPI:

```cpp
ICM42688P imu;

// For I2C
imu.begin(BUS_I2C);

// For SPI (specify CS pin)
imu.begin(BUS_SPI, 17);
```

### 2. Reading Data (FIFO)

The library reads 20-bit data packets from the FIFO to ensure high precision.

```cpp
float ax, ay, az, gx, gy, gz;

if (imu.readFIFO(ax, ay, az, gx, gy, gz)) {
    // ax, ay, az in g
    // gx, gy, gz in deg/s
    Serial.print("Accel X: "); Serial.println(ax);
}
```

### 3. Calibration & Offsets

The library supports two types of offsets:
1.  **Hardware Offsets**: Written to the sensor's registers. The sensor subtracts these values from the raw data *before* it reaches the FIFO.
2.  **Software Offsets**: Subtracted by the library *after* reading data from the FIFO.

#### Automatic Calibration

**Gyroscope:**
Calibrates gyro bias while the sensor is stationary. Writes results to **Hardware Offsets**.

```cpp
imu.autoCalibrateGyro(1000); // Collects 1000 samples
```

**Accelerometer (6-Point):**
Starts an interactive calibration wizard via Serial. Requires placing the sensor in 6 different positions.
- Computes bias -> writes to **Hardware Offsets**.
- Computes scale factor -> writes to **Software Scale**.

```cpp
imu.autoCalibrateAccel();
// Follow instructions on Serial Monitor
```

#### Manual Offset Management

**Hardware Offsets (Sensor Registers):**
```cpp
// Set hardware offsets (bias)
imu.setGyroHardwareOffset(0.5, -0.2, 0.1);
imu.setAccelHardwareOffset(0.01, -0.05, 0.00);

// Get currently cached hardware offsets
float ox, oy, oz;
imu.getAccelHardwareOffset(ox, oy, oz);
```

**Software Offsets & Scale (Library Processing):**
```cpp
// Set software offsets (bias)
imu.setGyroSoftwareOffset(0.5, -0.2, 0.1);
imu.setAccelSoftwareOffset(0.01, -0.05, 0.00);

// Set accelerometer scaling (matrix diagonal)
imu.setAccelSoftwareScale(1.001, 0.998, 1.005);
```

**Legacy Wrappers:**
For backward compatibility, these functions now map to the **Software** offset functions:
```cpp
imu.setGyroOffset(x, y, z);   // -> setGyroSoftwareOffset
imu.setAccelOffset(x, y, z);  // -> setAccelSoftwareOffset
```

### 4. Configuring Output Data Rate (ODR)

Set the Output Data Rate (ODR) for both accelerometer and gyroscope.

```cpp
imu.setODR(ODR_500HZ);
```

**Available ODR options:**
- `ODR_32KHZ`, `ODR_16KHZ`, `ODR_8KHZ`, `ODR_4KHZ`, `ODR_2KHZ`, `ODR_1KHZ`
- `ODR_500HZ` (default), `ODR_200HZ`, `ODR_100HZ`, `ODR_50HZ`, `ODR_25HZ`, `ODR_12_5HZ`

*Note: Higher ODR provides more data but increases power consumption and FIFO overflow risk if not read fast enough.*

---

## Examples

The library includes the following example sketches in the `examples/` folder:

- **ICM42688_I2C**:
  Initialize IMU over I2C, set ODR, and read FIFO data.

- **ICM42688_SPI_FIFO**:
  Initialize IMU over SPI, set ODR, and read FIFO data.

- **ICM42688_calibration**:
  Interactive tool for calibrating the sensor.
  - Send `'g'` via Serial to auto-calibrate Gyro.
  - Send `'a'` via Serial to start 6-point Accelerometer calibration.
  - Generates code snippets to copy-paste into your `setup()`.

---

## License

MIT License. See the LICENSE file for details.
