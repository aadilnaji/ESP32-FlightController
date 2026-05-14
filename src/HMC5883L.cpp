#include "HMC5883L.h"

HMC5883L mag;
float magX_offset = 0.0f;
float magY_offset = 0.0f;
float magZ_offset = 0.0f;

// ===== HMC5883L =====

HMC5883L::HMC5883L() {
    address = HMC5883L_ADDRESS;
    gainScale = 1.0f / 1090.0f;

    offsetX = offsetY = offsetZ = 0.0f;
    scaleX = scaleY = scaleZ = 1.0f;
}

bool HMC5883L::begin() {
    if (!isConnected()) {
        Serial.println("HMC5883L not found!");
        return false;
    }

    // 8 samples averaged, 75Hz, normal measurement
    writeRegister(HMC5883L_REG_CONFIG_A, 0x78);
    setGain(HMC5883L_GAIN_1090);
    setMode(HMC5883L_MODE_CONTINUOUS);
    delay(10);

    Serial.printf("HMC5883L initialized at address 0x%02X\n", address);
    return true;
}

void HMC5883L::setSampleRate(uint8_t rate) {
    uint8_t config = readRegister(HMC5883L_REG_CONFIG_A);
    config = (config & 0x9F) | rate;  // preserve averaging bits
    writeRegister(HMC5883L_REG_CONFIG_A, config);
}

void HMC5883L::setGain(uint8_t gain) {
    writeRegister(HMC5883L_REG_CONFIG_B, gain);

    switch (gain) {
        case HMC5883L_GAIN_1370: gainScale = 1.0f / 1370.0f; break;
        case HMC5883L_GAIN_1090: gainScale = 1.0f / 1090.0f; break;
        case HMC5883L_GAIN_820:  gainScale = 1.0f / 820.0f;  break;
        case HMC5883L_GAIN_660:  gainScale = 1.0f / 660.0f;  break;
        case HMC5883L_GAIN_440:  gainScale = 1.0f / 440.0f;  break;
        case HMC5883L_GAIN_390:  gainScale = 1.0f / 390.0f;  break;
        case HMC5883L_GAIN_330:  gainScale = 1.0f / 330.0f;  break;
        case HMC5883L_GAIN_230:  gainScale = 1.0f / 230.0f;  break;
    }
}

void HMC5883L::setMode(uint8_t mode) {
    writeRegister(HMC5883L_REG_MODE, mode);
}

void HMC5883L::setOffsets(float x, float y, float z) {
    offsetX = x; offsetY = y; offsetZ = z;
}

void HMC5883L::setScales(float x, float y, float z) {
    scaleX = x; scaleY = y; scaleZ = z;
}

bool HMC5883L::readRaw(int16_t& mx, int16_t& my, int16_t& mz) {
    Wire.beginTransmission(address);
    Wire.write(HMC5883L_REG_DATA_X_H);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(address, (uint8_t)6) != 6) {
        return false;
    }

    // HMC5883L outputs X, Z, Y (not X, Y, Z)
    mx = (Wire.read() << 8) | Wire.read();
    mz = (Wire.read() << 8) | Wire.read();
    my = (Wire.read() << 8) | Wire.read();

    return true;
}

bool HMC5883L::read(float& mx, float& my, float& mz) {
    int16_t rawX, rawY, rawZ;

    if (!readRaw(rawX, rawY, rawZ)) {
        return false;
    }

    mx = (rawX - offsetX) * scaleX;
    my = (rawY - offsetY) * scaleY;
    mz = (rawZ - offsetZ) * scaleZ;

    return true;
}

void HMC5883L::calibrate(uint32_t duration_ms) {
    Serial.println("Magnetometer calibration starting...");
    Serial.println("Rotate the sensor in all directions (figure 8 pattern)");

    int16_t minX = 32767, maxX = -32767;
    int16_t minY = 32767, maxY = -32767;
    int16_t minZ = 32767, maxZ = -32767;

    uint32_t startTime = millis();
    uint32_t samples = 0;

    while (millis() - startTime < duration_ms) {
        int16_t x, y, z;
        if (readRaw(x, y, z)) {
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
            if (z < minZ) minZ = z;
            if (z > maxZ) maxZ = z;
            samples++;
        }

        if (samples % 100 == 0) {
            Serial.printf("  Samples: %d, X: %d to %d, Y: %d to %d, Z: %d to %d\n",
                         samples, minX, maxX, minY, maxY, minZ, maxZ);
        }

        delay(10);
    }

    // Hard iron offsets: center of min/max
    offsetX = (maxX + minX) / 2.0f;
    offsetY = (maxY + minY) / 2.0f;
    offsetZ = (maxZ + minZ) / 2.0f;

    // Soft iron scales: normalize span to average
    float avgDelta = ((maxX - minX) + (maxY - minY) + (maxZ - minZ)) / 3.0f;
    scaleX = avgDelta / (maxX - minX);
    scaleY = avgDelta / (maxY - minY);
    scaleZ = avgDelta / (maxZ - minZ);

    Serial.println("\nCalibration complete!");
    Serial.printf("  Offsets: X=%.1f, Y=%.1f, Z=%.1f\n", offsetX, offsetY, offsetZ);
    Serial.printf("  Scales: X=%.3f, Y=%.3f, Z=%.3f\n", scaleX, scaleY, scaleZ);
    Serial.printf("  Total samples: %d\n", samples);
}

bool HMC5883L::isConnected() {
    // ID registers should read "H43"
    Wire.beginTransmission(address);
    Wire.write(HMC5883L_REG_ID_A);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(address, (uint8_t)3) != 3) {
        return false;
    }

    char id[4] = {0};
    id[0] = Wire.read();
    id[1] = Wire.read();
    id[2] = Wire.read();

    return (id[0] == 'H' && id[1] == '4' && id[2] == '3');
}

float HMC5883L::getHeading(float mx, float my) {
    float heading = atan2(my, mx) * 180.0f / PI;
    if (heading < 0) {
        heading += 360.0f;
    }
    return heading;
}

void HMC5883L::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t HMC5883L::readRegister(uint8_t reg) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(address, (uint8_t)1);
    return Wire.read();
}

// ===== GLOBAL FACADE =====

void hmc5883l_init() {
    mag.begin();
}

void hmc5883l_calibrate() {
    mag.calibrate(30000);
}

void hmc5883l_set_offsets(float x, float y, float z) {
    mag.setOffsets(x, y, z);
}

void hmc5883l_set_scales(float x, float y, float z) {
    mag.setScales(x, y, z);
}

void hmc5883l_get_calibration(float& ox, float& oy, float& oz, float& sx, float& sy, float& sz) {
    ox = mag.offsetX;
    oy = mag.offsetY;
    oz = mag.offsetZ;
    sx = mag.scaleX;
    sy = mag.scaleY;
    sz = mag.scaleZ;
}

bool hmc5883l_read(float& mx, float& my, float& mz) {
    return mag.read(mx, my, mz);
}
