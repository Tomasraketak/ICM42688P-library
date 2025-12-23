/*
 * Příklad 3: SPI Komunikace s FIFO, ODR a Kalibrací
 * * Toto je produkční šablona.
 * 1. Inicializuje SPI (rychlejší přenos).
 * 2. Nastaví ODR na 200 Hz (nebo více).
 * 3. Aplikuje kalibrační data (musíte vyplnit své hodnoty).
 * 4. Čte FIFO buffer co nejrychleji.
 */

#include "ICM42688P_voltino.h"

// Definice CS pinu (pro Raspberry Pi Pico defaultně GP17)
const uint8_t CS_PIN = 17;

ICM42688P IMU;

void setup() {
  Serial.begin(115200);
  // while (!Serial); // Odkomentovat, pokud chcete čekat na USB Serial

  // 1. Inicializace přes SPI
  if (!IMU.begin(BUS_SPI, CS_PIN)) {
    Serial.println("Chyba SPI komunikace! Zkontrolujte zapojeni.");
    while (1);
  }
  Serial.println("ICM-42688-P uspesne pripojeno pres SPI.");

  // 2. Nastavení ODR (Output Data Rate)
  // Možnosti: ODR_32KHZ až ODR_12_5HZ.
  // Pro běžné použití je ideální 100Hz - 500Hz.
  IMU.setODR(ODR_200HZ);


  // =============================================================
  // 3. SEKVENCE KALIBRACE (ZDE VLOŽTE HODNOTY Z PŘÍKLADU 1)
  // =============================================================
  
  // A) Akcelerometr (Bias + Scale)
  // Nahraďte 0.00 naměřenými hodnotami:
  // Příklad: IMU.setAccelHardwareOffset(0.021, -0.015, 0.005);
  IMU.setAccelHardwareOffset(0.0000, 0.0000, 0.0000); 
  IMU.setAccelSoftwareScale(1.0000, 1.0000, 1.0000);

  // B) Gyroskop (Bias)
  // Gyro se nejlépe kalibruje při každém startu, pokud je zařízení v klidu.
  // Pokud víte, že bude v pohybu, použijte setGyroHardwareOffset() s pevnými čísly.
  
  Serial.println("Kalibruji gyro (prosim nehýbat)...");
  IMU.autoCalibrateGyro(500); // Rychlá kalibrace při startu (500 vzorků)
  // Alternativně, pokud znáte hodnoty:
  // IMU.setGyroHardwareOffset(1.23, -0.55, 0.12);

  Serial.println("Nastaveni dokonceno. Startuji smycku.");
}

void loop() {
  float ax, ay, az, gx, gy, gz;

  // Čtení FIFO
  // Smyčka while zajistí, že přečteme všechna data, která se nahromadila v bufferu
  while (IMU.readFIFO(ax, ay, az, gx, gy, gz)) {
    
    // Zde probíhá zpracování dat (např. AHRS filtr, ukládání na SD kartu)
    // ...
    
    // Výpis dat (zpomalený, abychom nezahltili Serial Monitor)
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
      Serial.print("A [g]: ");
      Serial.print(ax, 3); Serial.print(", ");
      Serial.print(ay, 3); Serial.print(", ");
      Serial.print(az, 3);
      
      Serial.print(" | G [dps]: ");
      Serial.print(gx, 2); Serial.print(", ");
      Serial.print(gy, 2); Serial.print(", ");
      Serial.println(gz, 2);
      
      lastPrint = millis();
    }
  }
}