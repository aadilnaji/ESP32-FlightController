#include "PID.h"

// ===== PIDController =====

PIDController::PIDController() :
    kp(0.0f), ki(0.0f), kd(0.0f), kf(0.0f),
    iLimit(0.3f), outputLimit(500.0f),
    pScale(1.0f), iScale(1.0f), dScale(1.0f), fScale(1.0f),
    pTerm(0.0f), iTerm(0.0f), dTerm(0.0f), fTerm(0.0f),
    error(0.0f), iTermError(0.0f),
    itermRelaxBase(0.0f), itermRelaxFactor(1.0f),
    pFilterEnabled(false), dNotchEnabled(false),
    dLowpass1Enabled(false), dLowpass2Enabled(false),
    fFilterEnabled(false), itermRelaxEnabled(false),
    rate(1000.0f), dt(0.001f),
    prevMeasurement(0.0f), prevSetpoint(0.0f), prevError(0.0f),
    initialized(false), outputSaturated(false), lastOutput(0.0f),
    itermRelaxCutoff(40.0f),
    lastSetpoint(0.0f), lastMeasurement(0.0f) {}

void PIDController::init(float loopRateHz, const PIDConfig& config) {
    init(loopRateHz,
         config.pFilterHz, config.pFilterEnabled,
         config.dNotchHz, config.dNotchQ, config.dNotchEnabled,
         config.dLowpass1Hz, config.dLowpass1Enabled,
         config.dLowpass2Hz, config.dLowpass2Enabled,
         config.fFilterHz, config.fFilterEnabled,
         config.itermRelaxHz, config.itermRelaxEnabled, config.itermRelaxCutoff);

    setGains(config.kp, config.ki, config.kd, config.kf);
    setLimits(config.iLimit, config.outputLimit);
}

void PIDController::init(float loopRateHz,
                         float pFilterHz, bool pFilterEn,
                         float dNotchHz, float dNotchQ, bool dNotchEn,
                         float dFilter1Hz, bool dFilter1En,
                         float dFilter2Hz, bool dFilter2En,
                         float fFilterHz, bool fFilterEn,
                         float iRelaxHz, bool iRelaxEn, float iRelaxCutoff) {
    rate = loopRateHz;
    dt = 1.0f / rate;

    // Zero every state field so soft-reset doesn't carry stale values into the new run
    pTerm = 0.0f;
    iTerm = 0.0f;
    dTerm = 0.0f;
    fTerm = 0.0f;
    prevMeasurement = 0.0f;
    prevSetpoint = 0.0f;
    prevError = 0.0f;
    error = 0.0f;
    iTermError = 0.0f;
    outputSaturated = false;
    lastOutput = 0.0f;
    itermRelaxFactor = 1.0f;
    itermRelaxBase = 0.0f;
    lastSetpoint = 0.0f;
    lastMeasurement = 0.0f;

    pScale = 1.0f;
    iScale = 1.0f;
    dScale = 1.0f;
    fScale = 1.0f;

    pFilterEnabled = pFilterEn;
    dNotchEnabled = dNotchEn;
    dLowpass1Enabled = dFilter1En;
    dLowpass2Enabled = dFilter2En;
    fFilterEnabled = fFilterEn;
    itermRelaxEnabled = iRelaxEn;
    itermRelaxCutoff = iRelaxCutoff;

    pTermFilter.init(pFilterHz, rate, pFilterEn);
    dTermNotchFilter.initNotch(dNotchHz, rate, dNotchQ, dNotchEn);
    dTermFilter1.initLowpass(dFilter1Hz, rate, 0.707f, dFilter1En);
    dTermFilter2.initLowpass(dFilter2Hz, rate, 0.707f, dFilter2En);
    fTermFilter.init(fFilterHz, rate, fFilterEn);
    itermRelaxFilter.init(iRelaxHz, rate, iRelaxEn);

    initialized = false;
}

void PIDController::setGains(float _kp, float _ki, float _kd, float _kf) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
    kf = _kf;
}

void PIDController::setLimits(float _iLimit, float _outputLimit) {
    iLimit = _iLimit;
    outputLimit = _outputLimit;
}

void PIDController::setFeedForwardScale(float scale) {
    fScale = constrain(scale, 0.0f, 2.0f);
}

void PIDController::resetIterm() {
    iTerm = 0.0f;
}

void PIDController::reset() {
    iTerm = 0.0f;
    pTerm = 0.0f;
    dTerm = 0.0f;
    fTerm = 0.0f;
    prevMeasurement = 0.0f;
    prevSetpoint = 0.0f;
    prevError = 0.0f;
    error = 0.0f;
    iTermError = 0.0f;
    outputSaturated = false;
    lastOutput = 0.0f;
    itermRelaxFactor = 1.0f;
    itermRelaxBase = 0.0f;
    lastSetpoint = 0.0f;
    lastMeasurement = 0.0f;
    initialized = false;

    pTermFilter.reset();
    dTermNotchFilter.reset();
    dTermFilter1.reset();
    dTermFilter2.reset();
    fTermFilter.reset();
    itermRelaxFilter.reset();
}

float PIDController::update(float setpoint, float measurement) {
    lastSetpoint = setpoint;
    lastMeasurement = measurement;

    error = setpoint - measurement;

    // ===== P =====
    pTerm = kp * pScale * error;
    if (pFilterEnabled) {
        pTerm = pTermFilter.update(pTerm);
    }

    // ===== I (with relax + anti-windup) =====
    iTermError = error;

    if (itermRelaxEnabled && ki > 0.0f) {
        float setpointFiltered = itermRelaxFilter.update(setpoint);
        float setpointChange = fabsf(setpoint - setpointFiltered);

        itermRelaxBase = setpointChange;
        itermRelaxFactor = fmaxf(0.0f, 1.0f - setpointChange / itermRelaxCutoff);

        // Relax only when I-term would grow in the same direction
        bool iTermIncreasing = (iTerm > 0.0f && iTermError > 0.0f) ||
                               (iTerm < 0.0f && iTermError < 0.0f);
        if (iTermIncreasing) {
            iTermError *= itermRelaxFactor;
        }
    }

    if (ki > 0.0f && iScale > 0.0f) {
        if (!outputSaturated) {  // anti-windup
            iTerm += ki * iScale * iTermError * dt;
            iTerm = constrain(iTerm, -iLimit, iLimit);
        }
    } else {
        iTerm = 0.0f;
    }

    // ===== D on measurement (avoids setpoint-kick) =====
    if (kd > 0.0f && dScale > 0.0f && initialized) {
        float measurementDelta = prevMeasurement - measurement;
        dTerm = kd * dScale * measurementDelta * rate;

        // Filter order: notch -> lowpass1 -> lowpass2
        if (dNotchEnabled) {
            dTerm = dTermNotchFilter.update(dTerm);
        }
        if (dLowpass1Enabled) {
            dTerm = dTermFilter1.update(dTerm);
        }
        if (dLowpass2Enabled) {
            dTerm = dTermFilter2.update(dTerm);
        }
    } else {
        dTerm = 0.0f;
    }

    // ===== F (feed-forward) =====
    if (kf > 0.0f && fScale > 0.0f && initialized) {
        float setpointDelta = setpoint - prevSetpoint;
        fTerm = kf * fScale * setpointDelta * rate;
        if (fFilterEnabled) {
            fTerm = fTermFilter.update(fTerm);
        }
    } else {
        fTerm = 0.0f;
    }

    prevMeasurement = measurement;
    prevSetpoint = setpoint;
    prevError = error;
    initialized = true;

    float output = pTerm + iTerm + dTerm + fTerm;

    outputSaturated = (fabsf(output) >= outputLimit);
    lastOutput = constrain(output, -outputLimit, outputLimit);

    return lastOutput;
}

String PIDController::getFilterStatus() const {
    String s = "";
    s += pFilterEnabled ? "P" : "-";
    s += dNotchEnabled ? "N" : "-";
    s += dLowpass1Enabled ? "L1" : "--";
    s += dLowpass2Enabled ? "L2" : "--";
    s += fFilterEnabled ? "F" : "-";
    s += itermRelaxEnabled ? "R" : "-";
    return s;
}

// ===== RateCurve =====

static const float RC_RATE_INCREMENTAL = 14.54f;

static inline float power3(float x) { return x * x * x; }
static inline float constrainf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

RateCurve::RateCurve() {
    for (int i = 0; i < 3; i++) {
        rcRate[i] = 100.0f;
        superRate[i] = 70.0f;
        expo[i] = 20.0f;
        rateLimit[i] = 500.0f;
    }
}

void RateCurve::setRates(int axis, float rc, float super, float exp, float limit) {
    if (axis >= 0 && axis < 3) {
        rcRate[axis] = rc;
        superRate[axis] = super;
        expo[axis] = exp;
        rateLimit[axis] = limit;
    }
}

float RateCurve::getRate(int axis, float stickInput) const {
    stickInput = constrainf(stickInput, -0.995f, 0.995f);
    float rcCommandfAbs = fabsf(stickInput);
    float rcCommandf = stickInput;

    if (expo[axis] > 0) {
        float expof = expo[axis] / 100.0f;
        rcCommandf = rcCommandf * power3(rcCommandfAbs) * expof + rcCommandf * (1.0f - expof);
    }

    float rcRateVal = rcRate[axis] / 100.0f;
    if (rcRateVal > 2.0f) {
        rcRateVal += RC_RATE_INCREMENTAL * (rcRateVal - 2.0f);
    }
    float angleRate = 200.0f * rcRateVal * rcCommandf;

    if (superRate[axis] > 0) {
        float superFactor = superRate[axis] / 100.0f;
        float rcSuperfactor = 1.0f / constrainf(1.0f - (rcCommandfAbs * superFactor), 0.01f, 1.0f);
        angleRate *= rcSuperfactor;
    }

    return constrainf(angleRate, -rateLimit[axis], rateLimit[axis]);
}
