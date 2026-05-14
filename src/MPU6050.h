#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

/** MPU6050 IMU driver (gyro + accel + temp). */

// ===== I2C =====
#define MPU6050_ADDRESS       0x68
#define MPU6050_ADDRESS_ALT   0x69

// ===== REGISTERS =====
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_FIFO_EN       0x23
#define MPU6050_REG_INT_PIN_CFG   0x37
#define MPU6050_REG_INT_ENABLE    0x38
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_TEMP_OUT_H    0x41
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_REG_SIGNAL_PATH   0x68
#define MPU6050_REG_USER_CTRL     0x6A
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_PWR_MGMT_2    0x6C
#define MPU6050_REG_WHO_AM_I      0x75

// ===== GYRO RANGE =====
#define MPU6050_GYRO_FS_250       0x00
#define MPU6050_GYRO_FS_500       0x08
#define MPU6050_GYRO_FS_1000      0x10
#define MPU6050_GYRO_FS_2000      0x18

// ===== ACCEL RANGE =====
#define MPU6050_ACCEL_FS_2G       0x00
#define MPU6050_ACCEL_FS_4G       0x08
#define MPU6050_ACCEL_FS_8G       0x10
#define MPU6050_ACCEL_FS_16G      0x18

// ===== DLPF =====
#define MPU6050_DLPF_256HZ        0x00
#define MPU6050_DLPF_188HZ        0x01
#define MPU6050_DLPF_98HZ         0x02
#define MPU6050_DLPF_42HZ         0x03
#define MPU6050_DLPF_20HZ         0x04
#define MPU6050_DLPF_10HZ         0x05
#define MPU6050_DLPF_5HZ          0x06

struct IMURawData {
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    int16_t temp;
};

struct IMUData {
    float accelX, accelY, accelZ;  // g
    float gyroX, gyroY, gyroZ;     // deg/s
    float temperature;             // Celsius
};

struct IMUCalibration {
    float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
    float accelOffsetX, accelOffsetY, accelOffsetZ;
    float accelScaleX, accelScaleY, accelScaleZ;
};

class MPU6050 {
public:
    MPU6050();

    bool begin(int sda = 21, int scl = 22, uint32_t i2cSpeed = 400000);

    void setGyroRange(uint8_t range);     // MPU6050_GYRO_FS_*
    void setAccelRange(uint8_t range);    // MPU6050_ACCEL_FS_*
    void setDLPF(uint8_t dlpf);           // MPU6050_DLPF_*
    void setSampleRateDivider(uint8_t divider);

    /** Sensor must be stationary. */
    void calibrateGyro(int samples = 1000, int delayMs = 1);

    /** Sensor must be on a level surface. */
    void calibrateAccel(int samples = 500, int delayMs = 2);

    void setGyroOffsets(float x, float y, float z);
    void setAccelOffsets(float x, float y, float z);

    IMURawData readRaw();
    IMUData read();

    /** Lower-overhead read used in the hot loop. */
    bool readFast(float& ax, float& ay, float& az,
                  float& gx, float& gy, float& gz, float& temp);

    bool isConnected();
    uint8_t getDeviceID();

    IMUCalibration getCalibration() const { return cal; }
    void setCalibration(const IMUCalibration& calibration) { cal = calibration; }

private:
    uint8_t address;
    IMUCalibration cal;

    float gyroScale;   // LSB -> deg/s
    float accelScale;  // LSB -> g

    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length);
};

// ===== GLOBAL INSTANCE =====
extern MPU6050 imu;

void mpu6050_init();
void mpu6050_calibrate();
void mpu6050_calibrate_gyro();
void mpu6050_calibrate_accel();
void mpu6050_set_accel_offsets(float x, float y, float z);
IMUCalibration mpu6050_get_calibration();

bool mpu6050_read(float& ax, float& ay, float& az,
                  float& gx, float& gy, float& gz, float& temp);

#endif
