/*
 * Příklad 1: Kalibrační Nástroj (Calibration Tool)
 * * Tento skript slouží k naměření offsetů a scaling faktorů.
 * Výsledné hodnoty si zkopírujte a vložte do funkce setup() ve vašem hlavním programu.
 * * Instrukce:
 * 'g' -> Kalibrace gyra (Senzor musí být v naprostém klidu!)
 * 'a' -> Kalibrace akcelerometru (Budete vyzváni k položení senzoru do 6 poloh)
 * 'x' -> Reset (Smaže všechny HW offsety v čipu - první pomoc při divných datech)
 */

#include "ICM42688P_voltino.h"

// Definice CS pinu pro SPI (Raspberry Pi Pico default: 17)
// Pokud používáte I2C, změňte begin(BUS_SPI...) na begin(BUS_I2C) níže.
const uint8_t CS_PIN = 17;

ICM42688P IMU;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  // Inicializace (zde SPI, pro I2C upravte parametry)
  if (!IMU.begin(BUS_SPI, CS_PIN)) {
    Serial.println("IMU not found!");
    while (1);
  }
  
  Serial.println("ICM-42688-P Connected.");
  
  // Bezpečnostní reset čipu při startu
  //IMU.resetHardwareOffsets();
  //Serial.println("Hardware offsets cleared (Factory reset state).");
  IMU.setODR(ODR_500HZ); 

  Serial.println("--------------------------------------");
  Serial.println("Ready to calibrate.");
  Serial.println("Send 'g' to calibrate GYRO.");
  Serial.println("Send 'a' to calibrate ACCELEROMETER.");
  Serial.println("Send 'x' to RESET sensor offsets.");
  Serial.println("--------------------------------------");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    
    if (c == 'g') {
      // Spustí kalibraci gyra (1000 vzorků)
      // Funkce sama vypíše výsledný kód do Serial Monitoru
      IMU.autoCalibrateGyro(1000);
    } 
    else if (c == 'a') {
      // Spustí interaktivní 6-bodovou kalibraci akcelerometru
      // Funkce vás provede procesem a na konci vypíše kód
      IMU.autoCalibrateAccel();
    }
    else if (c == 'x') {
      IMU.resetHardwareOffsets();
      Serial.println("Hardwarové registry vymazány (Offset = 0).");
    }
  }
}