#include "ICM42688P_voltino.h"

ICM42688P::ICM42688P()
: _bus(BUS_I2C), _csPin(17), _spiSettings(10000000, MSBFIRST, SPI_MODE3),
  _odr(ODR_500HZ),
  _gOX(0), _gOY(0), _gOZ(0),
  _aOX(0), _aOY(0), _aOZ(0),
  _aSx(1.0f), _aSy(1.0f), _aSz(1.0f), // Default scale je 1.0 (žádná změna)
  _hwAccelBiasX(0), _hwAccelBiasY(0), _hwAccelBiasZ(0) {}

bool ICM42688P::begin(ICM_BUS busType, uint8_t csPin) {
  _bus = busType;
  _csPin = csPin;

  if (_bus == BUS_I2C) {
    Wire.begin();
  } else {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    SPI.begin();
    for (int i = 0; i < 3; i++) {
      digitalWrite(_csPin, LOW);
      delay(1);
      digitalWrite(_csPin, HIGH);
      delay(1);
    }
  }

  setBank(0);
  uint8_t who = readReg(0x75);
  if (who != 0x47 && who != 0x98) return false;

  writeReg(0x00, 0x01); // Reset
  delay(100);

  uint8_t intConfig1 = readReg(0x64);
  writeReg(0x64, intConfig1 & ~(1 << 4));
  writeReg(0x4E, 0x0F); // LN mode
  writeReg(0x4F, (0 << 5) | (uint8_t)_odr);
  writeReg(0x50, (0 << 5) | (uint8_t)_odr);
  writeReg(0x5F, (1 << 4) | (1 << 3) | (1 << 1) | (1 << 0)); 
  writeReg(0x16, 0x40); // stream mode

  return true;
}

void ICM42688P::setODR(ICM_ODR odr) {
  _odr = odr;
  setBank(0);
  writeReg(0x4F, (0 << 5) | (uint8_t)_odr);
  writeReg(0x50, (0 << 5) | (uint8_t)_odr);
}

void ICM42688P::setBank(uint8_t bank) {
  writeReg(REG_BANK_SEL, bank);
}

bool ICM42688P::readFIFO(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint16_t fifoCount = ((uint16_t)readReg(0x2E) << 8) | readReg(0x2F);
  if (fifoCount < 20) return false;

  uint8_t packet[20];
  readRegs(0x30, packet, 20);

  uint8_t header = packet[0];
  if ((header & 0x80) || !(header & 0x10)) return false;

  int32_t ax_r = (int32_t)((int8_t)packet[1] << 12 | packet[2] << 4 | (packet[17] >> 4));
  ax_r = (ax_r << 12) >> 12;
  int32_t ay_r = (int32_t)((int8_t)packet[3] << 12 | packet[4] << 4 | (packet[18] >> 4));
  ay_r = (ay_r << 12) >> 12;
  int32_t az_r = (int32_t)((int8_t)packet[5] << 12 | packet[6] << 4 | (packet[19] >> 4));
  az_r = (az_r << 12) >> 12;

  int32_t gx_r = (int32_t)((int8_t)packet[7] << 12 | packet[8] << 4 | (packet[17] & 0x0F));
  gx_r = (gx_r << 12) >> 12;
  int32_t gy_r = (int32_t)((int8_t)packet[9] << 12 | packet[10] << 4 | (packet[18] & 0x0F));
  gy_r = (gy_r << 12) >> 12;
  int32_t gz_r = (int32_t)((int8_t)packet[11] << 12 | packet[12] << 4 | (packet[19] & 0x0F));
  gz_r = (gz_r << 12) >> 12;

  // 1. Získání základní hodnoty (HW bias už je odečten čipem, pokud je nastaven)
  float raw_ax = (ax_r / 8192.0f);
  float raw_ay = (ay_r / 8192.0f);
  float raw_az = (az_r / 8192.0f);

  // 2. Aplikace SW offsetů (pokud nějaké jsou)
  raw_ax -= _aOX;
  raw_ay -= _aOY;
  raw_az -= _aOZ;

  // 3. Aplikace SW Scaling (toto opravuje "šišatost" / citlivost)
  ax = raw_ax * _aSx;
  ay = raw_ay * _aSy;
  az = raw_az * _aSz;

  // Gyro (zatím jen SW offset)
  gx = (gx_r / 131.0f) - _gOX;
  gy = (gy_r / 131.0f) - _gOY;
  gz = (gz_r / 131.0f) - _gOZ;

  return true;
}

// --- HARDWARE OFFSET API ---
void ICM42688P::setGyroHardwareOffset(float ox, float oy, float oz) {
  setBank(4);
  int16_t ox_reg = (int16_t)(ox * -32.0f);
  int16_t oy_reg = (int16_t)(oy * -32.0f);
  int16_t oz_reg = (int16_t)(oz * -32.0f);
  writeReg(0x0B, ox_reg & 0xFF); writeReg(0x0C, (ox_reg >> 8) & 0xFF);
  writeReg(0x0D, oy_reg & 0xFF); writeReg(0x0E, (oy_reg >> 8) & 0xFF);
  writeReg(0x0F, oz_reg & 0xFF); writeReg(0x10, (oz_reg >> 8) & 0xFF);
  setBank(0);
}

void ICM42688P::setAccelHardwareOffset(float ox, float oy, float oz) {
  _hwAccelBiasX = ox; _hwAccelBiasY = oy; _hwAccelBiasZ = oz;
  setBank(4);
  int16_t ox_reg = (int16_t)(ox * -2000.0f);
  int16_t oy_reg = (int16_t)(oy * -2000.0f);
  int16_t oz_reg = (int16_t)(oz * -2000.0f);
  writeReg(0x11, ox_reg & 0xFF); writeReg(0x12, (ox_reg >> 8) & 0xFF);
  writeReg(0x13, oy_reg & 0xFF); writeReg(0x14, (oy_reg >> 8) & 0xFF);
  writeReg(0x15, oz_reg & 0xFF); writeReg(0x16, (oz_reg >> 8) & 0xFF);
  setBank(0);
}

void ICM42688P::getAccelHardwareOffset(float &ox, float &oy, float &oz) {
  ox = _hwAccelBiasX; oy = _hwAccelBiasY; oz = _hwAccelBiasZ;
}

// --- SW API ---
void ICM42688P::setGyroSoftwareOffset(float ox, float oy, float oz) { _gOX = ox; _gOY = oy; _gOZ = oz; }
void ICM42688P::setAccelSoftwareOffset(float ox, float oy, float oz) { _aOX = ox; _aOY = oy; _aOZ = oz; }
void ICM42688P::setAccelSoftwareScale(float sx, float sy, float sz) { _aSx = sx; _aSy = sy; _aSz = sz; }
void ICM42688P::getAccelSoftwareScale(float &sx, float &sy, float &sz) { sx = _aSx; sy = _aSy; sz = _aSz; }

// --- KALIBRACE ---
void ICM42688P::autoCalibrateGyro(uint16_t samples) {
  Serial.println("GYRO CALIBRATION...");
  setGyroHardwareOffset(0, 0, 0); _gOX = 0; _gOY = 0; _gOZ = 0;
  delay(200);
  double sumX = 0, sumY = 0, sumZ = 0;
  uint16_t count = 0;
  for (uint16_t i = 0; i < samples; i++) {
    float ax, ay, az, gx, gy, gz;
    if (readFIFO(ax, ay, az, gx, gy, gz)) {
      sumX += gx; sumY += gy; sumZ += gz; count++;
    }
    delayMicroseconds(2000);
  }
  if (count > 0) setGyroHardwareOffset(sumX/count, sumY/count, sumZ/count);
}

// --- POKROČILÁ KALIBRACE (BIAS + SCALE) ---
void ICM42688P::autoCalibrateAccel() {
  Serial.println(F("\n=== 6-POINT ACCEL CALIBRATION (BIAS + SCALE) ==="));
  Serial.println(F("Place sensor in 6 different orientations (roughly +X, -X, +Y, -Y, +Z, -Z)."));
  
  // 1. Reset stávajících kalibrací
  setAccelHardwareOffset(0, 0, 0);
  setAccelSoftwareScale(1.0, 1.0, 1.0);
  _aOX = 0; _aOY = 0; _aOZ = 0;
  
  struct Vector { float x, y, z; };
  Vector points[6];

  // 2. Sběr dat
  for (int i = 0; i < 6; i++) {
    Serial.print(F("\nPosition ")); Serial.print(i + 1); Serial.println(F("/6 -> Send 'y'"));
    while (Serial.available()) Serial.read();
    while (!Serial.available()); Serial.read();

    Serial.println(F("Measuring..."));
    double sumX = 0, sumY = 0, sumZ = 0;
    int count = 0;
    unsigned long start = millis();
    while (millis() - start < 1500) {
      float ax, ay, az, gx, gy, gz;
      if (readFIFO(ax, ay, az, gx, gy, gz)) {
        sumX += ax; sumY += ay; sumZ += az; count++;
      }
      delay(2);
    }
    points[i].x = sumX / count; points[i].y = sumY / count; points[i].z = sumZ / count;
    Serial.print(F("Raw: ")); Serial.print(points[i].x); Serial.print(", ");
    Serial.print(points[i].y); Serial.print(", "); Serial.println(points[i].z);
  }

  Serial.println(F("\nCalculating Bias & Scale..."));

  // 3. Optimalizace (Gradient Descent) pro 6 parametrů:
  // bx, by, bz (Bias) a sx, sy, sz (Scale)
  // Cíl: || (raw - bias) * scale || = 1.0
  
  float bx = 0, by = 0, bz = 0;
  float sx = 1, sy = 1, sz = 1;
  float learningRate = 0.05;

  for (int iter = 0; iter < 800; iter++) {
    float dbx = 0, dby = 0, dbz = 0;
    float dsx = 0, dsy = 0, dsz = 0;

    for (int i = 0; i < 6; i++) {
      // Aplikace aktuálního modelu
      float adjX = (points[i].x - bx) * sx;
      float adjY = (points[i].y - by) * sy;
      float adjZ = (points[i].z - bz) * sz;
      
      float radius = sqrt(adjX*adjX + adjY*adjY + adjZ*adjZ);
      float error = radius - 1.0f; // Chceme radius 1.0

      // Derivace chybové funkce (zjednodušené gradienty)
      // dE/db = dE/dr * dr/d_adj * d_adj/db
      float common = error / radius; // (r - 1) / r

      // Gradienty pro Bias
      dbx += -2.0f * common * adjX * sx; // * sx vzniklo z vnitřní derivace
      dby += -2.0f * common * adjY * sy;
      dbz += -2.0f * common * adjZ * sz;

      // Gradienty pro Scale
      // d_adj/ds = (raw - b) ... což je zhruba adj / s
      dsx += 2.0f * common * adjX * (points[i].x - bx);
      dsy += 2.0f * common * adjY * (points[i].y - by);
      dsz += 2.0f * common * adjZ * (points[i].z - bz);
    }

    // Update vah
    bx -= learningRate * (dbx / 6.0f);
    by -= learningRate * (dby / 6.0f);
    bz -= learningRate * (dbz / 6.0f);
    
    sx -= learningRate * (dsx / 6.0f);
    sy -= learningRate * (dsy / 6.0f);
    sz -= learningRate * (dsz / 6.0f);

    if (iter % 100 == 0) learningRate *= 0.9; // Snižování learning rate
  }

  // 4. Aplikace a výpis
  Serial.println(F("Done!"));
  Serial.print(F("Bias (HW): ")); Serial.print(bx, 4); Serial.print(", "); Serial.print(by, 4); Serial.print(", "); Serial.println(bz, 4);
  Serial.print(F("Scale (SW): ")); Serial.print(sx, 4); Serial.print(", "); Serial.print(sy, 4); Serial.print(", "); Serial.println(sz, 4);

  // Zápis do systému
  setAccelHardwareOffset(bx, by, bz);
  setAccelSoftwareScale(sx, sy, sz);

  Serial.println(F("\n--- COPY TO SETUP() ---"));
  Serial.print(F("IMU.setAccelHardwareOffset(")); Serial.print(bx, 4); Serial.print(", "); Serial.print(by, 4); Serial.print(", "); Serial.println(bz, 4); Serial.print(");");
  Serial.print(F("\nIMU.setAccelSoftwareScale(")); Serial.print(sx, 4); Serial.print(", "); Serial.print(sy, 4); Serial.print(", "); Serial.println(sz, 4); Serial.print(");");
  Serial.println(F("\n-----------------------"));
}

// Low level beze změn...
uint8_t ICM42688P::readReg(uint8_t reg) {
  if (_bus == BUS_I2C) {
    Wire.beginTransmission(ICM_ADDR); Wire.write(reg); Wire.endTransmission(false);
    Wire.requestFrom(ICM_ADDR, (uint8_t)1); return Wire.available() ? Wire.read() : 0;
  } else {
    uint8_t val; SPI.beginTransaction(_spiSettings); digitalWrite(_csPin, LOW);
    SPI.transfer(reg | 0x80); val = SPI.transfer(0x00); digitalWrite(_csPin, HIGH); SPI.endTransaction(); return val;
  }
}
void ICM42688P::writeReg(uint8_t reg, uint8_t val) {
  if (_bus == BUS_I2C) {
    Wire.beginTransmission(ICM_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
  } else {
    SPI.beginTransaction(_spiSettings); digitalWrite(_csPin, LOW); SPI.transfer(reg); SPI.transfer(val); digitalWrite(_csPin, HIGH); SPI.endTransaction();
  }
}
void ICM42688P::readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  if (_bus == BUS_I2C) {
    Wire.beginTransmission(ICM_ADDR); Wire.write(reg); Wire.endTransmission(false); Wire.requestFrom(ICM_ADDR, len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) buf[i] = Wire.read();
  } else {
    SPI.beginTransaction(_spiSettings); digitalWrite(_csPin, LOW); SPI.transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00); digitalWrite(_csPin, HIGH); SPI.endTransaction();
  }
}