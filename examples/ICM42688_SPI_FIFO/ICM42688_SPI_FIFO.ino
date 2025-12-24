/*
 * Příklad 3: SPI Komunikace s FIFO, ODR a SW Kalibrací
 * * Tento příklad používá "Safe" verzi knihovny se softwarovými offsety.
 * Offsety se odečítají v procesoru, nikoliv v čipu.
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

  // DŮLEŽITÉ: Vymažeme případné staré/chybné kalibrace přímo v čipu,
  // abychom začali s čistým štítem.
  //IMU.resetHardwareOffsets();

  // 2. Nastavení ODR
  IMU.setODR(ODR_4KHZ);

  // =============================================================
  // 3. SEKVENCE KALIBRACE (SW OFFSETY)
  // =============================================================
  
  // A) Akcelerometr (Bias + Scale)
  // Používáme setAccelOffset (Softwarový) místo HardwareOffset
  // Vlož sem hodnoty, které ti vypsal Calibration Tool
  IMU.setAccelOffset(-0.0019, 0.0012, -0.0010); 
  IMU.setAccelScale(1.0061, 1.0033, 0.9996);

  // B) Gyroskop (Bias)
  // Možnost 1: Automatická kalibrace při každém startu (Doporučeno pro gyro)
  Serial.println("Kalibruji gyro (prosim nehýbat)...");
  IMU.autoCalibrateGyro(500); // Změří bias a uloží ho do SW proměnných
  
  // Možnost 2: Manuální nastavení (pokud znáš hodnoty a nechceš čekat)
  // Pokud toto odkomentuješ, zakomentuj autoCalibrateGyro výše.
  // IMU.setGyroOffset(0.61, -0.27, 0.16); 

  Serial.println("Nastaveni dokonceno. Startuji smycku.");
}

void loop() {
  float ax, ay, az, gx, gy, gz;

  // Čtení FIFO
  // readFIFO automaticky odečte nastavené SW offsety
  while (IMU.readFIFO(ax, ay, az, gx, gy, gz)) {
    
    // Výpis dat
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 50) {
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