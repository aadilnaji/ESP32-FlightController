#ifndef HMC5883L_H
#define HMC5883L_H

#include <Arduino.h>
#include <Wire.h>

/** HMC5883L 3-axis magnetometer driver. */

// ===== I2C =====
#define HMC5883L_ADDRESS        0x1E

// ===== REGISTERS =====
#define HMC5883L_REG_CONFIG_A   0x00
#define HMC5883L_REG_CONFIG_B   0x01
#define HMC5883L_REG_MODE       0x02
#define HMC5883L_REG_DATA_X_H   0x03
#define HMC5883L_REG_DATA_X_L   0x04
#define HMC5883L_REG_DATA_Z_H   0x05
#define HMC5883L_REG_DATA_Z_L   0x06
#define HMC5883L_REG_DATA_Y_H   0x07
#define HMC5883L_REG_DATA_Y_L   0x08
#define HMC5883L_REG_STATUS     0x09
#define HMC5883L_REG_ID_A       0x0A
#define HMC5883L_REG_ID_B       0x0B
#define HMC5883L_REG_ID_C       0x0C

// ===== SAMPLE RATE =====
#define HMC5883L_RATE_0_75      0x00
#define HMC5883L_RATE_1_5       0x04
#define HMC5883L_RATE_3         0x08
#define HMC5883L_RATE_7_5       0x0C
#define HMC5883L_RATE_15        0x10  // default
#define HMC5883L_RATE_30        0x14
#define HMC5883L_RATE_75        0x18

// ===== GAIN (range) =====
#define HMC5883L_GAIN_1370      0x00  // +/-0.88 Ga
#define HMC5883L_GAIN_1090      0x20  // +/-1.3 Ga (default)
#define HMC5883L_GAIN_820       0x40  // +/-1.9 Ga
#define HMC5883L_GAIN_660       0x60  // +/-2.5 Ga
#define HMC5883L_GAIN_440       0x80  // +/-4.0 Ga
#define HMC5883L_GAIN_390       0xA0  // +/-4.7 Ga
#define HMC5883L_GAIN_330       0xC0  // +/-5.6 Ga
#define HMC5883L_GAIN_230       0xE0  // +/-8.1 Ga

// ===== MODE =====
#define HMC5883L_MODE_CONTINUOUS 0x00
#define HMC5883L_MODE_SINGLE     0x01
#define HMC5883L_MODE_IDLE       0x02

class HMC5883L {
public:
    float offsetX, offsetY, offsetZ;
    float scaleX, scaleY, scaleZ;

    HMC5883L();

    bool begin();
    void setSampleRate(uint8_t rate);
    void setGain(uint8_t gain);
    void setMode(uint8_t mode);

    void setOffsets(float x, float y, float z);
    void setScales(float x, float y, float z);

    bool readRaw(int16_t& mx, int16_t& my, int16_t& mz);
    bool read(float& mx, float& my, float& mz);

    /** Rotate sensor in figure-8 for the given duration to capture hard/soft iron offsets. */
    void calibrate(uint32_t duration_ms = 30000);

    bool isConnected();

    /** Heading in degrees (0-360) from horizontal-plane mx/my. */
    float getHeading(float mx, float my);

private:
    uint8_t address;
    float gainScale;  // LSB -> uT

    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
};

// ===== GLOBAL INSTANCE =====
extern HMC5883L mag;

extern float magX_offset;
extern float magY_offset;
extern float magZ_offset;

void hmc5883l_init();
void hmc5883l_calibrate();
void hmc5883l_set_offsets(float x, float y, float z);
void hmc5883l_set_scales(float x, float y, float z);
void hmc5883l_get_calibration(float& ox, float& oy, float& oz, float& sx, float& sy, float& sz);
bool hmc5883l_read(float& mx, float& my, float& mz);

#endif
