/*
 * Příklad 1: Kalibrace IMU (ICM-42688-P)
 * Tento skript slouží k získání kalibračních hodnot (Offsety a Scaling).
 * * Instrukce:
 * 1. Nahrajte skript.
 * 2. Otevřete Serial Monitor (115200 baud).
 * 3. Pošlete 'g' pro kalibraci gyroskopu (senzor musí být v klidu).
 * 4. Pošlete 'a' pro kalibraci akcelerometru (budete vyzváni k 6 polohám).
 * 5. Zkopírujte vygenerovaný kód z výstupu do svého finálního programu.
 */

#include "ICM42688P_voltino.h"

// Vyberte komunikační sběrnici (odkomentujte jednu možnost)
// Pro I2C:
ICM42688P IMU;
// Pro SPI (CS pin 17 pro Pico):
// ICM42688P IMU; 
// const int CS_PIN = 17;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);

  Serial.println("Inicializace ICM-42688-P...");

  // Inicializace I2C
  if (!IMU.begin(BUS_I2C)) {
  
  // Inicializace SPI (pokud používáte SPI, odkomentujte toto a zakomentujte I2C výše)
  // if (!IMU.begin(BUS_SPI, CS_PIN)) {
    
    Serial.println("Chyba: Senzor nenalezen!");
    while (1);
  }

  Serial.println("Senzor pripraven.");
  Serial.println("----------------------------------------------");
  Serial.println("Prikazy:");
  Serial.println("  'g' -> Kalibrace Gyroskopu (nehybat se)");
  Serial.println("  'a' -> Kalibrace Akcelerometru (6 poloh)");
  Serial.println("----------------------------------------------");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'g') {
      // Kalibrace gyra - změří 1000 vzorků
      IMU.autoCalibrateGyro(1000);
    } 
    else if (c == 'a') {
      // Kalibrace akcelerometru - interaktivní průvodce
      IMU.autoCalibrateAccel();
    }
  }
}