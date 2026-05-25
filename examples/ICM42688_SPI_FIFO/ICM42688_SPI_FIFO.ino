/*
 * Example 2: SPI Communication with HIRES 20-BIT FIFO
 * Shows how to enable the High-Resolution 20-bit mode, set limits, 
 * and efficiently flush the hardware buffer.
 */

#include "ICM42688P_voltino.h"

// Adjust CS pin for your board (e.g., 17 for Raspberry Pi Pico, 10 for Arduino Uno)
const uint8_t CS_PIN = 17; 

// SPI Frequency. 10MHz is great for ESP32/RP2040. Use 4000000 for standard AVR Arduinos.
const uint32_t SPI_FREQ = 10000000; 

ICM42688P IMU;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Turn on debugging so we can see Auto-Bandwidth warnings
  IMU.setDebug(true);

  // Initialize via SPI
  if (!IMU.beginSPI(CS_PIN, SPI_FREQ)) {
    Serial.println("SPI Communication Error! Check connections.");
    while (1);
  }

  // Enable advanced 20-bit High-Resolution FIFO mode
  // Note: This automatically locks the sensor's physical limits to 16G and 2000dps
  IMU.setFIFOMode(FIFO_20BIT_HIRES);

  // Set Output Data Rate
  // If the requested ODR is too high for the SPI bus capacity, 
  // the library will automatically downgrade it to a safe level.
  IMU.setODR(ODR_1KHZ); 
  
  Serial.print("Actual Output Data Rate applied: ");
  Serial.print(IMU.getODRHz());
  Serial.println(" Hz");

  // Add your calculated offsets here...
  // IMU.setAccelOffset(...);
  
  Serial.println("Setup complete. Starting loop.");
}

void loop() {
  float ax, ay, az, gx, gy, gz;

  // The 'while' loop is critical when using FIFO!
  // It ensures we empty the hardware buffer rapidly if the MCU was busy doing other tasks.
  while (IMU.readIMU(ax, ay, az, gx, gy, gz)) {
    
    static unsigned long lastPrint = 0;
    // Print data to Serial only 10 times a second to prevent flooding the monitor
    if (millis() - lastPrint >= 100) { 
      lastPrint = millis();
      
      Serial.print("A: ");
      Serial.print(ax, 5); Serial.print(", "); // 5 decimal places to see 20-bit precision
      Serial.print(ay, 5); Serial.print(", ");
      Serial.print(az, 5);
      
      Serial.print(" | G: ");
      Serial.print(gx, 3); Serial.print(", ");
      Serial.print(gy, 3); Serial.print(", ");
      Serial.println(gz, 3);
    }
  }
}