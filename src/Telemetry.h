#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include "Config.h"

/** 50Hz snapshot of flight state, published by the flight task and consumed by web/serial. */
struct TelemetryData {
    float roll, pitch, yaw;
    float angleErrRoll, angleErrPitch;

    float gyroX, gyroY, gyroZ;
    float gyroXf, gyroYf, gyroZf;

    float accelX, accelY, accelZ;

    float magX, magY, magZ;
    bool magValid;

    float rateSetRoll, rateSetPitch, rateSetYaw;
    float rateErrRoll, rateErrPitch, rateErrYaw;

    float pTermRoll, pTermPitch, pTermYaw;
    float iTermRoll, iTermPitch, iTermYaw;
    float dTermRoll, dTermPitch, dTermYaw;
    float fTermRoll, fTermPitch, fTermYaw;
    float pidTotalRoll, pidTotalPitch, pidTotalYaw;

    float mixRoll, mixPitch, mixYaw;
    int throttle, m1, m2, m3, m4;
    bool motorSaturated;
    char saturationAxis;  // 'R', 'P', 'Y', or 0

    float rcRoll, rcPitch, rcYaw;
    int rcThrottle;
    int rcArmSwitch, rcModeSwitch;

    bool armed;
    FlightMode mode;
    bool rcConnected;
    bool fusionConverged;
    bool accelOk;
    bool magOk;
    bool airmodeActive;

    uint32_t loopTime;
    uint32_t loopMin, loopMax;
    uint32_t loopAvg;
    uint32_t loopJitter;
    uint32_t overruns;

    float tpaFactor;
    float trimRoll, trimPitch;

    bool headlessActive;
    float headlessRefYaw;
};

namespace Telemetry {
    /** Flight task -> mutex-protected store. Call at the publish rate (~50Hz). */
    void publish(const TelemetryData& snapshot);

    /** Mutex-protected copy of latest snapshot into the caller's buffer. */
    void snapshot(TelemetryData& out);

    /** Build the compact JSON payload served by the /telem endpoint. */
    String buildJson();
}

#endif
