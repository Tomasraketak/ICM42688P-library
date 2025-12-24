#include "ICM42688P_voltino.h"

ICM42688P::ICM42688P()
: _bus(BUS_I2C), _csPin(17), 
  _spiSettings(10000000, MSBFIRST, SPI_MODE3), // 10 MHz
  _odr(ODR_500HZ),
  _gOX(0), _gOY(0), _gOZ(0),
  _aOX(0), _aOY(0), _aOZ(0),
  _aSx(1.0f), _aSy(1.0f), _aSz(1.0f),
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
      digitalWrite(_csPin, LOW); delay(1); digitalWrite(_csPin, HIGH); delay(1);
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
  writeReg(0x16, 0x40); // Stream mode

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

// --- ČTENÍ FIFO (Původní funkční logika) ---
bool ICM42688P::readFIFO(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint16_t fifoCount = ((uint16_t)readReg(0x2E) << 8) | readReg(0x2F);
  if (fifoCount < 20) return false;

  uint8_t packet[20];
  readRegs(0x30, packet, 20);

  uint8_t header = packet[0];
  if ((header & 0x80) || !(header & 0x10)) return false; 

  int32_t ax_r = (int32_t)((int8_t)packet[1] << 12 | packet[2] << 4 | (packet[17] >> 4));
  ax_r = (ax_r << 12) >> 12; 
  ax_r >>= 2; 

  int32_t ay_r = (int32_t)((int8_t)packet[3] << 12 | packet[4] << 4 | (packet[18] >> 4));
  ay_r = (ay_r << 12) >> 12; 
  ay_r >>= 2;

  int32_t az_r = (int32_t)((int8_t)packet[5] << 12 | packet[6] << 4 | (packet[19] >> 4));
  az_r = (az_r << 12) >> 12; 
  az_r >>= 2;

  int32_t gx_r = (int32_t)((int8_t)packet[7] << 12 | packet[8] << 4 | (packet[17] & 0x0F));
  gx_r = (gx_r << 12) >> 12; 
  gx_r >>= 1; 

  int32_t gy_r = (int32_t)((int8_t)packet[9] << 12 | packet[10] << 4 | (packet[18] & 0x0F));
  gy_r = (gy_r << 12) >> 12; 
  gy_r >>= 1;

  int32_t gz_r = (int32_t)((int8_t)packet[11] << 12 | packet[12] << 4 | (packet[19] & 0x0F));
  gz_r = (gz_r << 12) >> 12; 
  gz_r >>= 1;

  // Aplikace SW korekcí (HW korekce jsou již aplikovány čipem na raw data)
  float raw_ax = (ax_r / 8192.0f) - _aOX;
  float raw_ay = (ay_r / 8192.0f) - _aOY;
  float raw_az = (az_r / 8192.0f) - _aOZ;

  ax = raw_ax * _aSx;
  ay = raw_ay * _aSy;
  az = raw_az * _aSz;

  gx = (gx_r / 131.0f) - _gOX;
  gy = (gy_r / 131.0f) - _gOY;
  gz = (gz_r / 131.0f) - _gOZ;

  return true;
}

// --- Wrappers (nyní mapují na SW offsety pro bezpečnost) ---
void ICM42688P::setGyroOffset(float ox, float oy, float oz) { setGyroSoftwareOffset(ox, oy, oz); }
void ICM42688P::setAccelOffset(float ox, float oy, float oz) { setAccelSoftwareOffset(ox, oy, oz); }
void ICM42688P::getGyroOffset(float &ox, float &oy, float &oz) { ox = _gOX; oy = _gOY; oz = _gOZ; }
void ICM42688P::getAccelOffset(float &ox, float &oy, float &oz) { ox = _aOX; oy = _aOY; oz = _aOZ; }

// --- SOFTWARE API ---
void ICM42688P::setGyroSoftwareOffset(float ox, float oy, float oz) { _gOX = ox; _gOY = oy; _gOZ = oz; }
void ICM42688P::setAccelSoftwareOffset(float ox, float oy, float oz) { _aOX = ox; _aOY = oy; _aOZ = oz; }
void ICM42688P::setAccelSoftwareScale(float sx, float sy, float sz) { _aSx = sx; _aSy = sy; _aSz = sz; }
void ICM42688P::getAccelSoftwareScale(float &sx, float &sy, float &sz) { sx = _aSx; sy = _aSy; sz = _aSz; }

// --- HARDWARE API (Bit-Packing Fix) ---

// Pomocná funkce pro 12-bit limit
int16_t clamp12bit_fix(int32_t val) {
  if (val > 2047) return 2047;
  if (val < -2048) return -2048;
  return (int16_t)val;
}

void ICM42688P::setGyroHardwareOffset(float ox, float oy, float oz) {
  setBank(4);
  
  // 1 LSB = 1/32 dps.
  int16_t gx_lsb = clamp12bit_fix((int32_t)(ox * 32.0f));
  int16_t gy_lsb = clamp12bit_fix((int32_t)(oy * 32.0f));
  int16_t gz_lsb = clamp12bit_fix((int32_t)(oz * 32.0f));

  // Opatrný zápis do registrů (Read-Modify-Write pro sdílené bity)
  uint8_t reg0F_current = readReg(0x0F); // Obsahuje horní bity Accel X a Gyro Z
  
  uint8_t reg0B = gx_lsb & 0xFF;
  // Gyro Y (11:8) a Gyro X (11:8)
  uint8_t reg0C = ((gy_lsb >> 8) & 0x0F) << 4 | ((gx_lsb >> 8) & 0x0F);
  uint8_t reg0D = gy_lsb & 0xFF;
  uint8_t reg0E = gz_lsb & 0xFF;
  // Zachováme Accel X (horní 4), přepíšeme Gyro Z (dolní 4)
  uint8_t reg0F = (reg0F_current & 0xF0) | ((gz_lsb >> 8) & 0x0F);

  writeReg(0x0B, reg0B);
  writeReg(0x0C, reg0C);
  writeReg(0x0D, reg0D);
  writeReg(0x0E, reg0E);
  writeReg(0x0F, reg0F);

  setBank(0);
}

void ICM42688P::setAccelHardwareOffset(float ox, float oy, float oz) {
  _hwAccelBiasX = ox; _hwAccelBiasY = oy; _hwAccelBiasZ = oz;
  setBank(4);

  // 1 LSB = 0.5 mg -> 2000 LSB/g
  int16_t ax_lsb = clamp12bit_fix((int32_t)(ox * 2000.0f));
  int16_t ay_lsb = clamp12bit_fix((int32_t)(oy * 2000.0f));
  int16_t az_lsb = clamp12bit_fix((int32_t)(oz * 2000.0f));

  // 0x0F: Horní 4 bity Accel X (7:4) | Gyro Z (3:0)
  // 0x12: Horní 4 bity Accel Z (7:4) | Accel Y (3:0)
  
  uint8_t reg0F_current = readReg(0x0F); 
  
  // Zachováme Gyro Z (dolní 4), přepíšeme Accel X (horní 4)
  uint8_t reg0F = (reg0F_current & 0x0F) | (((ax_lsb >> 8) & 0x0F) << 4);
  
  uint8_t reg10 = ax_lsb & 0xFF;
  uint8_t reg11 = ay_lsb & 0xFF;
  uint8_t reg12 = ((az_lsb >> 8) & 0x0F) << 4 | ((ay_lsb >> 8) & 0x0F);
  uint8_t reg13 = az_lsb & 0xFF;

  writeReg(0x0F, reg0F);
  writeReg(0x10, reg10);
  writeReg(0x11, reg11);
  writeReg(0x12, reg12);
  writeReg(0x13, reg13);

  setBank(0);
}

void ICM42688P::getAccelHardwareOffset(float &ox, float &oy, float &oz) {
  ox = _hwAccelBiasX; oy = _hwAccelBiasY; oz = _hwAccelBiasZ;
}

// --- KALIBRACE ---
void ICM42688P::autoCalibrateGyro(uint16_t samples) {
  Serial.println("GYRO CALIBRATION (HW)... Keep still.");
  // Vyčistit vše
  setGyroHardwareOffset(0, 0, 0); 
  _gOX = 0; _gOY = 0; _gOZ = 0;
  
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

  if (count > 0) {
    float avgBiasX = (float)(sumX / count);
    float avgBiasY = (float)(sumY / count);
    float avgBiasZ = (float)(sumZ / count);

    // Zápis do hardwaru!
    setGyroHardwareOffset(avgBiasX, avgBiasY, avgBiasZ);
    Serial.println("Done. HW Offsets written.");
  }
}

void ICM42688P::autoCalibrateAccel() {
  Serial.println(F("\n=== 6-POINT ACCEL CALIBRATION (HW Bias + SW Scale) ==="));
  
  setAccelHardwareOffset(0, 0, 0);
  setAccelSoftwareScale(1.0, 1.0, 1.0);
  _aOX = 0; _aOY = 0; _aOZ = 0;
  
  struct Vector { float x, y, z; };
  Vector points[6];

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

  Serial.println(F("\nCalculating..."));
  float bx = 0, by = 0, bz = 0;
  float sx = 1, sy = 1, sz = 1;
  float learningRate = 0.05;

  for (int iter = 0; iter < 1000; iter++) {
    float dbx = 0, dby = 0, dbz = 0;
    float dsx = 0, dsy = 0, dsz = 0;

    for (int i = 0; i < 6; i++) {
      float adjX = (points[i].x - bx) * sx;
      float adjY = (points[i].y - by) * sy;
      float adjZ = (points[i].z - bz) * sz;
      float radius = sqrt(adjX*adjX + adjY*adjY + adjZ*adjZ);
      float error = radius - 1.0f;
      float common = error / radius;

      dbx += -2.0f * common * adjX * sx;
      dby += -2.0f * common * adjY * sy;
      dbz += -2.0f * common * adjZ * sz;

      dsx += 2.0f * common * adjX * (points[i].x - bx);
      dsy += 2.0f * common * adjY * (points[i].y - by);
      dsz += 2.0f * common * adjZ * (points[i].z - bz);
    }
    bx -= learningRate * (dbx / 6.0f); by -= learningRate * (dby / 6.0f); bz -= learningRate * (dbz / 6.0f);
    sx -= learningRate * (dsx / 6.0f); sy -= learningRate * (dsy / 6.0f); sz -= learningRate * (dsz / 6.0f);
    if (iter % 100 == 0) learningRate *= 0.9;
  }

  // Bias zapíšeme do HW (čip to odečte)
  setAccelHardwareOffset(bx, by, bz);
  // Scale zapíšeme do SW (knihovna to vynásobí)
  setAccelSoftwareScale(sx, sy, sz);

  Serial.println(F("\n--- COPY TO SETUP() ---"));
  Serial.print(F("IMU.setAccelHardwareOffset(")); Serial.print(bx, 4); Serial.print(", "); Serial.print(by, 4); Serial.print(", "); Serial.println(bz, 4); Serial.print(");");
  Serial.print(F("\nIMU.setAccelSoftwareScale(")); Serial.print(sx, 4); Serial.print(", "); Serial.print(sy, 4); Serial.print(", "); Serial.println(sz, 4); Serial.print(");");
  Serial.println(F("\n-----------------------"));
}

// Low level
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
    Wire.beginTransmission(ICM_ADDR); Wire.write(reg); Wire.endTransmission(false);
    Wire.requestFrom(ICM_ADDR, len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) buf[i] = Wire.read();
  } else {
    SPI.beginTransaction(_spiSettings); digitalWrite(_csPin, LOW); SPI.transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00); digitalWrite(_csPin, HIGH); SPI.endTransaction();
  }
}