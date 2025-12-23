/*
 * Příklad 2: Základní čtení přes I2C
 * Ověření funkčnosti senzoru a 20-bitového parsování.
 */

#include "ICM42688P_voltino.h"

ICM42688P IMU;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Startuji I2C komunikaci...");
  
  if (!IMU.begin(BUS_I2C)) {
    Serial.println("Senzor nenalezen na I2C (zkontroluj adresu 0x68)!");
    while (1);
  }

  // Nastavíme rychlost vzorkování
  IMU.setODR(ODR_50HZ); 
  Serial.println("Senzor OK. Ctu data...");
}

void loop() {
  float ax, ay, az, gx, gy, gz;

  // Funkce readFIFO vrátí true, pokud je v zásobníku nový kompletní packet
  if (IMU.readFIFO(ax, ay, az, gx, gy, gz)) {
    Serial.print("Accel [g]: ");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az);
    
    Serial.print("\t | Gyro [dps]: ");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
    
    delay(20); // Jen pro čitelnost výpisu
  }
}