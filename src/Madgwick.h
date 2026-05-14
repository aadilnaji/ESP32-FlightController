#ifndef MADGWICK_H
#define MADGWICK_H

#include <Arduino.h>
#include <math.h>

/** Madgwick AHRS sensor fusion (gradient descent). Algorithm credit: Sebastian O.H. Madgwick */

class MadgwickAHRS {
public:
    float q0, q1, q2, q3;     // quaternion
    float beta;               // filter gain
    float sampleFreq;
    float invSampleFreq;

    MadgwickAHRS();
    MadgwickAHRS(float beta);

    void begin(float sampleFrequency);
    void setBeta(float b) { beta = b; }
    float getBeta() const { return beta; }
    void reset();

    /** Gyro+accel update. Gyro in rad/s, accel normalized internally. */
    void update(float gx, float gy, float gz, float ax, float ay, float az);

    /** Gyro+accel+mag update. */
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz);

    /** Gyro-only update (dead reckoning). */
    void updateIMU(float gx, float gy, float gz);

    float getRoll() const;
    float getPitch() const;
    float getYaw() const;

    float getRollRad() const;
    float getPitchRad() const;
    float getYawRad() const;

    void getQuaternion(float* w, float* x, float* y, float* z) const;

private:
    static float invSqrt(float x);
};

/** Wrapper exposing a MahonyFusion-compatible API for drop-in use in the flight loop. */
class MadgwickFusion {
public:
    struct FusedOutput {
        float roll_deg, pitch_deg, yaw_deg;
        float roll_rad, pitch_rad, yaw_rad;
        float q[4];
        bool data_valid;
    };

    MadgwickFusion(float betaStartup = 0.5f, float betaFlight = 0.1f);

    void begin(float sampleFrequency);
    void setBeta(float b);
    float getBeta() const { return ahrs.getBeta(); }

    // Mahony-API compatibility shim: maps proportional gain to beta
    void setGains(float kp, float ki = 0.0f) { setBeta(kp * 0.2f); }

    void reset();

    /** Main fusion step. Gyro deg/s, accel m/s^2 or g. */
    FusedOutput update(float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float mx, float my, float mz,
                       bool useAccel = true, bool useMag = false);

private:
    MadgwickAHRS ahrs;
    float betaStartup;
    float betaFlight;
    bool initialized;
    uint64_t lastUpdate_us;
};

#endif
