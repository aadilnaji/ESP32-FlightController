#include "Madgwick.h"

// ===== MadgwickAHRS =====

MadgwickAHRS::MadgwickAHRS() {
    beta = 0.1f;
    sampleFreq = 1000.0f;
    invSampleFreq = 0.001f;
    reset();
}

MadgwickAHRS::MadgwickAHRS(float b) {
    beta = b;
    sampleFreq = 1000.0f;
    invSampleFreq = 0.001f;
    reset();
}

void MadgwickAHRS::begin(float sampleFrequency) {
    sampleFreq = sampleFrequency;
    invSampleFreq = 1.0f / sampleFrequency;
}

void MadgwickAHRS::reset() {
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
}

// Quake III fast inverse square root
float MadgwickAHRS::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void MadgwickAHRS::updateIMU(float gx, float gy, float gz) {
    float qa, qb, qc;

    gx *= (0.5f * invSampleFreq);
    gy *= (0.5f * invSampleFreq);
    gz *= (0.5f * invSampleFreq);

    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void MadgwickAHRS::update(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Apply accel correction only when measurement is valid
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
        q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3;

        // Gradient descent correction
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    q0 += qDot1 * invSampleFreq; q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq; q3 += qDot4 * invSampleFreq;

    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

void MadgwickAHRS::update(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz;
    float _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Fall back to IMU-only when mag is invalid
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        update(gx, gy, gz, ax, ay, az);
        return;
    }

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        _2q0mx = 2.0f * q0 * mx; _2q0my = 2.0f * q0 * my; _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2; _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
        q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
        q2q2 = q2 * q2; q2q3 = q2 * q3; q3q3 = q3 * q3;

        // Earth's magnetic field reference direction
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx; _4bz = 2.0f * _2bz;

        // Gradient descent correction
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        qDot1 -= beta * s0; qDot2 -= beta * s1; qDot3 -= beta * s2; qDot4 -= beta * s3;
    }

    q0 += qDot1 * invSampleFreq; q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq; q3 += qDot4 * invSampleFreq;

    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

float MadgwickAHRS::getRoll() const {
    return getRollRad() * 57.29577951f;
}

float MadgwickAHRS::getPitch() const {
    return getPitchRad() * 57.29577951f;
}

float MadgwickAHRS::getYaw() const {
    return getYawRad() * 57.29577951f;
}

float MadgwickAHRS::getRollRad() const {
    return atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
}

float MadgwickAHRS::getPitchRad() const {
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        return copysignf(M_PI / 2.0f, sinp);
    }
    return asinf(sinp);
}

float MadgwickAHRS::getYawRad() const {
    return atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

void MadgwickAHRS::getQuaternion(float* w, float* x, float* y, float* z) const {
    *w = q0;
    *x = q1;
    *y = q2;
    *z = q3;
}

// ===== MadgwickFusion =====

MadgwickFusion::MadgwickFusion(float betaStart, float betaFlt)
    : betaStartup(betaStart), betaFlight(betaFlt), initialized(false), lastUpdate_us(0) {
    ahrs.setBeta(betaStartup);
}

void MadgwickFusion::begin(float sampleFrequency) {
    ahrs.begin(sampleFrequency);
    initialized = true;
}

void MadgwickFusion::setBeta(float b) {
    ahrs.setBeta(b);
}

void MadgwickFusion::reset() {
    ahrs.reset();
    lastUpdate_us = 0;
    initialized = false;
}

MadgwickFusion::FusedOutput MadgwickFusion::update(float ax, float ay, float az,
                                                    float gx, float gy, float gz,
                                                    float mx, float my, float mz,
                                                    bool useAccel, bool useMag) {
    FusedOutput out;

    uint64_t now_us = micros();
    float dt;
    if (!initialized) {
        dt = 1.0f / 250.0f;
        initialized = true;
    } else {
        dt = (now_us - lastUpdate_us) / 1000000.0f;
        if (dt < 0.0005f) dt = 0.0005f;
        if (dt > 0.1f) dt = 0.1f;
    }
    lastUpdate_us = now_us;

    // Track real loop timing rather than assuming the nominal rate
    ahrs.begin(1.0f / dt);

    float gx_rad = gx * 0.01745329251f;
    float gy_rad = gy * 0.01745329251f;
    float gz_rad = gz * 0.01745329251f;

    if (useAccel) {
        if (useMag) {
            ahrs.update(gx_rad, gy_rad, gz_rad, ax, ay, az, mx, my, mz);
        } else {
            ahrs.update(gx_rad, gy_rad, gz_rad, ax, ay, az);
        }
    } else {
        ahrs.updateIMU(gx_rad, gy_rad, gz_rad);
    }

    out.roll_deg = ahrs.getRoll();
    out.pitch_deg = ahrs.getPitch();
    out.yaw_deg = ahrs.getYaw();
    out.roll_rad = ahrs.getRollRad();
    out.pitch_rad = ahrs.getPitchRad();
    out.yaw_rad = ahrs.getYawRad();
    ahrs.getQuaternion(&out.q[0], &out.q[1], &out.q[2], &out.q[3]);
    out.data_valid = true;

    return out;
}
