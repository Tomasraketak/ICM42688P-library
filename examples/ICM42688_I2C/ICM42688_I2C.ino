/*
 * Příklad 2: Základní čtení přes I2C s aplikací SW kalibrace
 * * Tento skript inicializuje senzor, vyčistí staré (možná chybné) HW offsety
 * a čte data z FIFO bufferu.
 */

#include "ICM42688P_voltino.h"

ICM42688P IMU;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Startuji I2C komunikaci...");
  
  // Adresa je defaultně 0x68. Pro I2C není potřeba CS pin.
  if (!IMU.begin(BUS_I2C)) {
    Serial.println("Senzor nenalezen na I2C (zkontroluj adresu 0x68)!");
    while (1);
  }

  // DŮLEŽITÉ: Vymažeme případné staré HW offsety v čipu, aby nezkreslovaly data
  //IMU.resetHardwareOffsets();

  // Nastavíme rychlost vzorkování
  //IMU.setODR(ODR_50HZ); 

  // --- SEM VLOŽ KALIBRAČNÍ HODNOTY (Pokud je máš změřené) ---
  // Příklad:
  // IMU.setAccelOffset(0.02, -0.01, 0.05);
  // IMU.setAccelScale(1.001, 0.999, 1.002);
  // IMU.setGyroOffset(0.5, -0.2, 0.1);

  Serial.println("Senzor OK. Ctu data...");
}

void loop() {
  float ax, ay, az, gx, gy, gz;

  // Funkce readFIFO vrátí true, pokud je v zásobníku nový kompletní packet
  // Automaticky aplikuje výše nastavené SW offsety
  if (IMU.readFIFO(ax, ay, az, gx, gy, gz)) {
    
    // Zpomalíme výpis pro čitelnost
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