#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include "Config.h"

/** Single sample fed to the logger from the flight loop. */
struct FlightFrame {
    uint64_t timestamp_us;
    float gx, gy, gz;          // raw gyro deg/s
    float gxf, gyf, gzf;        // filtered gyro deg/s
    float roll, pitch, yaw;     // fused angles deg
    float pidR, pidP, pidY;     // pre-TPA PID outputs
    float cmdR, cmdP, cmdY;     // post-TPA mixer inputs
    int throttle, m1, m2, m3, m4;
};

namespace Logger {
    void start(int seconds);
    void stop();
    void dump();

    bool isActive();   // hot-path predicate; safe to read without locking
    bool hasData();    // true once a logging window has completed (dump() will print)

    void sample(const FlightFrame& f);
}

#endif
