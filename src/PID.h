#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include "Filters.h"

/** PID controller with D-on-measurement, feed-forward, I-term relax, anti-windup, and per-term filtering. */

enum ItermRelaxMode {
    ITERM_RELAX_OFF = 0,
    ITERM_RELAX_RP,
    ITERM_RELAX_RPY,
    ITERM_RELAX_RP_INC,
    ITERM_RELAX_RPY_INC
};

struct PIDConfig {
    float kp;
    float ki;
    float kd;
    float kf;
    float iLimit;
    float outputLimit;

    bool pFilterEnabled;
    float pFilterHz;

    bool dNotchEnabled;
    float dNotchHz;
    float dNotchQ;

    bool dLowpass1Enabled;
    float dLowpass1Hz;

    bool dLowpass2Enabled;
    float dLowpass2Hz;

    bool fFilterEnabled;
    float fFilterHz;

    bool itermRelaxEnabled;
    float itermRelaxHz;
    float itermRelaxCutoff;

    PIDConfig() :
        kp(1.0f), ki(0.0f), kd(0.0f), kf(0.0f),
        iLimit(0.3f), outputLimit(500.0f),
        pFilterEnabled(false), pFilterHz(100.0f),
        dNotchEnabled(false), dNotchHz(150.0f), dNotchQ(5.0f),
        dLowpass1Enabled(true), dLowpass1Hz(100.0f),
        dLowpass2Enabled(false), dLowpass2Hz(200.0f),
        fFilterEnabled(false), fFilterHz(100.0f),
        itermRelaxEnabled(true), itermRelaxHz(15.0f), itermRelaxCutoff(40.0f) {}
};

class PIDController {
public:
    float kp, ki, kd, kf;
    float iLimit;
    float outputLimit;

    float pScale, iScale, dScale, fScale;

    // Last-computed term values, exposed for telemetry
    float pTerm, iTerm, dTerm, fTerm;
    float error;
    float iTermError;

    float itermRelaxBase;
    float itermRelaxFactor;

    bool pFilterEnabled;
    bool dNotchEnabled;
    bool dLowpass1Enabled;
    bool dLowpass2Enabled;
    bool fFilterEnabled;
    bool itermRelaxEnabled;

private:
    float rate;
    float dt;

    float prevMeasurement;
    float prevSetpoint;
    float prevError;
    bool initialized;

    bool outputSaturated;
    float lastOutput;

    float itermRelaxCutoff;

    PT1Filter pTermFilter;
    BiquadFilter dTermNotchFilter;
    BiquadFilter dTermFilter1;
    BiquadFilter dTermFilter2;
    PT1Filter fTermFilter;
    PT1Filter itermRelaxFilter;

    float lastSetpoint;
    float lastMeasurement;

public:
    PIDController();

    void init(float loopRateHz, const PIDConfig& config);

    void init(float loopRateHz,
              float pFilterHz, bool pFilterEn,
              float dNotchHz, float dNotchQ, bool dNotchEn,
              float dFilter1Hz, bool dFilter1En,
              float dFilter2Hz, bool dFilter2En,
              float fFilterHz, bool fFilterEn,
              float iRelaxHz, bool iRelaxEn, float iRelaxCutoff);

    void setGains(float _kp, float _ki, float _kd, float _kf = 0.0f);
    void setLimits(float _iLimit, float _outputLimit);
    void setFeedForwardScale(float scale);

    /** Reset I-term only; filter states preserved. */
    void resetIterm();

    /** Full reset including filter state. */
    void reset();

    float update(float setpoint, float measurement);

    bool isSaturated() const { return outputSaturated; }
    float getLastOutput() const { return lastOutput; }

    /** Compact filter-enable string for status display. */
    String getFilterStatus() const;

    // ===== TELEMETRY GETTERS =====
    float getLastPTerm() const { return pTerm; }
    float getLastITerm() const { return iTerm; }
    float getLastDTerm() const { return dTerm; }
    float getLastFTerm() const { return fTerm; }
    float getLastError() const { return error; }
    float getLastSetpoint() const { return lastSetpoint; }
    float getLastMeasurement() const { return lastMeasurement; }
    float getItermRelaxFactor() const { return itermRelaxFactor; }

    float getItermPercent() const {
        return (iLimit > 0) ? (fabsf(iTerm) / iLimit * 100.0f) : 0.0f;
    }

    bool isItermAtLimit() const {
        return fabsf(iTerm) >= (iLimit * 0.99f);
    }

    struct PIDTerms {
        float p, i, d, f;
        float total;
        float error;
        float setpoint;
        float measurement;
        bool saturated;
        bool iAtLimit;
        float iRelaxFactor;
    };

    PIDTerms getAllTerms() const {
        PIDTerms t;
        t.p = pTerm;
        t.i = iTerm;
        t.d = dTerm;
        t.f = fTerm;
        t.total = lastOutput;
        t.error = error;
        t.setpoint = lastSetpoint;
        t.measurement = lastMeasurement;
        t.saturated = outputSaturated;
        t.iAtLimit = isItermAtLimit();
        t.iRelaxFactor = itermRelaxFactor;
        return t;
    }
};

// ===== RATE CURVE =====

/** Stick input -> angular rate setpoint, per-axis (roll/pitch/yaw). */
class RateCurve {
public:
    float rcRate[3];     // center sensitivity
    float superRate[3];  // max rate at full stick
    float expo[3];       // center softness
    float rateLimit[3];  // hard cap (deg/s)

    RateCurve();

    /** axis: 0=Roll 1=Pitch 2=Yaw. rc 0-255 (typ 100), super 0-100, exp 0-100, limit in deg/s. */
    void setRates(int axis, float rc, float super, float exp, float limit);

    /** stickInput is -1.0..1.0; returns deg/s setpoint. */
    float getRate(int axis, float stickInput) const;
};

// ===== TPA (Throttle PID Attenuation) =====

class TPAController {
public:
    int breakpoint;     // throttle (us) where attenuation begins
    float scale;        // attenuation at full throttle (0-100%)
    bool enabled;

    TPAController() : breakpoint(1600), scale(30.0f), enabled(true) {}

    void setConfig(int bp, float sc) {
        breakpoint = bp;
        scale = constrain(sc, 0.0f, 100.0f);
    }

    /** Returns multiplier 0.1..1.0 applied to PID output above the breakpoint. */
    float getFactor(int throttleUs) const {
        if (!enabled || scale == 0.0f) return 1.0f;

        float t = constrain((float)throttleUs, (float)breakpoint, 2000.0f);
        float factor = 1.0f - (t - breakpoint) / (2000.0f - breakpoint) * (scale / 100.0f);

        return constrain(factor, 0.1f, 1.0f);
    }

    int getBreakpoint() const { return breakpoint; }
    float getScale() const { return scale; }
    bool isEnabled() const { return enabled; }
};

#endif
