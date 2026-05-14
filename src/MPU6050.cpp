#include "MPU6050.h"

MPU6050 imu;

// ===== MPU6050 =====

MPU6050::MPU6050() {
    address = MPU6050_ADDRESS;
    gyroScale = 1.0f / 131.0f;     // +/-250 deg/s default
    accelScale = 1.0f / 16384.0f;  // +/-2g default

    cal.gyroOffsetX = cal.gyroOffsetY = cal.gyroOffsetZ = 0.0f;
    cal.accelOffsetX = cal.accelOffsetY = cal.accelOffsetZ = 0.0f;
    cal.accelScaleX = cal.accelScaleY = cal.accelScaleZ = 1.0f;
}

bool MPU6050::begin(int sda, int scl, uint32_t i2cSpeed) {
    Wire.begin(sda, scl);
    Wire.setClock(i2cSpeed);

    if (!isConnected()) {
        address = MPU6050_ADDRESS_ALT;
        if (!isConnected()) {
            Serial.println("MPU6050 not found!");
            return false;
        }
    }

    writeRegister(MPU6050_REG_PWR_MGMT_1, 0x80);  // reset
    delay(100);
    writeRegister(MPU6050_REG_PWR_MGMT_1, 0x01);  // wake, PLL from X-gyro
    delay(10);

    writeRegister(MPU6050_REG_SMPLRT_DIV, 0x00);  // 1kHz sample rate
    setDLPF(MPU6050_DLPF_98HZ);                   // noise/latency balance
    setGyroRange(MPU6050_GYRO_FS_2000);           // headroom for acro
    setAccelRange(MPU6050_ACCEL_FS_8G);
    writeRegister(MPU6050_REG_FIFO_EN, 0x00);
    writeRegister(MPU6050_REG_INT_PIN_CFG, 0x20);

    delay(50);

    Serial.printf("MPU6050 initialized at address 0x%02X\n", address);
    Serial.printf("  Gyro scale: %.4f deg/s per LSB\n", gyroScale);
    Serial.printf("  Accel scale: %.6f g per LSB\n", accelScale);

    return true;
}

void MPU6050::setGyroRange(uint8_t range) {
    writeRegister(MPU6050_REG_GYRO_CONFIG, range);

    switch (range) {
        case MPU6050_GYRO_FS_250:  gyroScale = 1.0f / 131.0f; break;
        case MPU6050_GYRO_FS_500:  gyroScale = 1.0f / 65.5f;  break;
        case MPU6050_GYRO_FS_1000: gyroScale = 1.0f / 32.8f;  break;
        case MPU6050_GYRO_FS_2000: gyroScale = 1.0f / 16.4f;  break;
    }
}

void MPU6050::setAccelRange(uint8_t range) {
    writeRegister(MPU6050_REG_ACCEL_CONFIG, range);

    switch (range) {
        case MPU6050_ACCEL_FS_2G:  accelScale = 1.0f / 16384.0f; break;
        case MPU6050_ACCEL_FS_4G:  accelScale = 1.0f / 8192.0f;  break;
        case MPU6050_ACCEL_FS_8G:  accelScale = 1.0f / 4096.0f;  break;
        case MPU6050_ACCEL_FS_16G: accelScale = 1.0f / 2048.0f;  break;
    }
}

void MPU6050::setDLPF(uint8_t dlpf) {
    writeRegister(MPU6050_REG_CONFIG, dlpf & 0x07);
}

void MPU6050::setSampleRateDivider(uint8_t divider) {
    writeRegister(MPU6050_REG_SMPLRT_DIV, divider);
}

void MPU6050::calibrateGyro(int samples, int delayMs) {
    Serial.println("Calibrating gyroscope...");
    Serial.println("  Keep sensor stationary!");

    float sumX = 0, sumY = 0, sumZ = 0;
    int validSamples = 0;

    // Warm-up to discard transient readings
    for (int i = 0; i < 100; i++) {
        readRaw();
        delay(1);
    }

    for (int i = 0; i < samples; i++) {
        IMURawData raw = readRaw();
        sumX += raw.gyroX;
        sumY += raw.gyroY;
        sumZ += raw.gyroZ;
        validSamples++;
        delay(delayMs);

        if (i % 100 == 0) {
            Serial.printf("  Progress: %d/%d\n", i, samples);
        }
    }

    if (validSamples > 0) {
        cal.gyroOffsetX = (sumX / validSamples) * gyroScale;
        cal.gyroOffsetY = (sumY / validSamples) * gyroScale;
        cal.gyroOffsetZ = (sumZ / validSamples) * gyroScale;
    }

    Serial.printf("  Gyro offsets: X=%.2f Y=%.2f Z=%.2f deg/s\n",
                  cal.gyroOffsetX, cal.gyroOffsetY, cal.gyroOffsetZ);
}

void MPU6050::calibrateAccel(int samples, int delayMs) {
    Serial.println("Calibrating accelerometer...");
    Serial.println("  Keep sensor level!");

    float sumX = 0, sumY = 0, sumZ = 0;
    int validSamples = 0;

    for (int i = 0; i < 50; i++) {
        readRaw();
        delay(2);
    }

    for (int i = 0; i < samples; i++) {
        IMURawData raw = readRaw();
        sumX += raw.accelX;
        sumY += raw.accelY;
        sumZ += raw.accelZ;
        validSamples++;
        delay(delayMs);
    }

    if (validSamples > 0) {
        float avgX = (sumX / validSamples) * accelScale;
        float avgY = (sumY / validSamples) * accelScale;
        float avgZ = (sumZ / validSamples) * accelScale;

        cal.accelOffsetX = avgX;
        cal.accelOffsetY = avgY;
        cal.accelOffsetZ = avgZ - 1.0f;  // Z reads 1g when level
    }

    Serial.printf("  Accel offsets: X=%.4f Y=%.4f Z=%.4f g\n",
                  cal.accelOffsetX, cal.accelOffsetY, cal.accelOffsetZ);
}

void MPU6050::setGyroOffsets(float x, float y, float z) {
    cal.gyroOffsetX = x; cal.gyroOffsetY = y; cal.gyroOffsetZ = z;
}

void MPU6050::setAccelOffsets(float x, float y, float z) {
    cal.accelOffsetX = x; cal.accelOffsetY = y; cal.accelOffsetZ = z;
}

IMURawData MPU6050::readRaw() {
    IMURawData data;
    uint8_t buffer[14];

    readRegisters(MPU6050_REG_ACCEL_XOUT_H, buffer, 14);

    data.accelX = (int16_t)((buffer[0] << 8) | buffer[1]);
    data.accelY = (int16_t)((buffer[2] << 8) | buffer[3]);
    data.accelZ = (int16_t)((buffer[4] << 8) | buffer[5]);
    data.temp   = (int16_t)((buffer[6] << 8) | buffer[7]);
    data.gyroX  = (int16_t)((buffer[8] << 8) | buffer[9]);
    data.gyroY  = (int16_t)((buffer[10] << 8) | buffer[11]);
    data.gyroZ  = (int16_t)((buffer[12] << 8) | buffer[13]);

    return data;
}

IMUData MPU6050::read() {
    IMURawData raw = readRaw();
    IMUData data;

    data.accelX = (raw.accelX * accelScale - cal.accelOffsetX) * cal.accelScaleX;
    data.accelY = (raw.accelY * accelScale - cal.accelOffsetY) * cal.accelScaleY;
    data.accelZ = (raw.accelZ * accelScale - cal.accelOffsetZ) * cal.accelScaleZ;

    data.gyroX = raw.gyroX * gyroScale - cal.gyroOffsetX;
    data.gyroY = raw.gyroY * gyroScale - cal.gyroOffsetY;
    data.gyroZ = raw.gyroZ * gyroScale - cal.gyroOffsetZ;

    data.temperature = (raw.temp / 340.0f) + 36.53f;

    return data;
}

bool MPU6050::readFast(float& ax, float& ay, float& az,
                       float& gx, float& gy, float& gz, float& temp) {
    uint8_t buffer[14];

    Wire.beginTransmission(address);
    Wire.write(MPU6050_REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(address, (uint8_t)14) != 14) {
        return false;
    }

    for (int i = 0; i < 14; i++) {
        buffer[i] = Wire.read();
    }

    int16_t rawAx = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t rawAy = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t rawAz = (int16_t)((buffer[4] << 8) | buffer[5]);
    int16_t rawT  = (int16_t)((buffer[6] << 8) | buffer[7]);
    int16_t rawGx = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t rawGy = (int16_t)((buffer[10] << 8) | buffer[11]);
    int16_t rawGz = (int16_t)((buffer[12] << 8) | buffer[13]);

    ax = (rawAx * accelScale - cal.accelOffsetX) * cal.accelScaleX;
    ay = (rawAy * accelScale - cal.accelOffsetY) * cal.accelScaleY;
    az = (rawAz * accelScale - cal.accelOffsetZ) * cal.accelScaleZ;

    gx = rawGx * gyroScale - cal.gyroOffsetX;
    gy = rawGy * gyroScale - cal.gyroOffsetY;
    gz = rawGz * gyroScale - cal.gyroOffsetZ;

    temp = (rawT / 340.0f) + 36.53f;

    return true;
}

bool MPU6050::isConnected() {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() != 0) {
        return false;
    }

    uint8_t whoami = getDeviceID();
    return (whoami == 0x68 || whoami == 0x98);
}

uint8_t MPU6050::getDeviceID() {
    return readRegister(MPU6050_REG_WHO_AM_I);
}

void MPU6050::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t MPU6050::readRegister(uint8_t reg) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(address, (uint8_t)1);
    return Wire.read();
}

void MPU6050::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(address, length);

    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = Wire.read();
    }
}

// ===== GLOBAL FACADE =====

void mpu6050_init() {
    imu.begin(21, 22, 400000);
}

void mpu6050_calibrate() {
    imu.calibrateGyro(1000, 1);
    imu.calibrateAccel(1000, 2);
}

void mpu6050_calibrate_gyro() {
    imu.calibrateGyro(1000, 1);
}

void mpu6050_calibrate_accel() {
    imu.calibrateAccel(1000, 2);
}

void mpu6050_set_accel_offsets(float x, float y, float z) {
    imu.setAccelOffsets(x, y, z);
}

IMUCalibration mpu6050_get_calibration() {
    return imu.getCalibration();
}

bool mpu6050_read(float& ax, float& ay, float& az,
                  float& gx, float& gy, float& gz, float& temp) {
    return imu.readFast(ax, ay, az, gx, gy, gz, temp);
}
