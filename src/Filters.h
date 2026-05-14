#ifndef FILTERS_H
#define FILTERS_H

#include <Arduino.h>
#include <math.h>

/** IIR filter library: PT1/PT2/PT3 lowpass, biquad, slew limiter, gyro filter bank. 
 *  Filter topology and gyro/D-term filter bank inspired by Betaflight.
*/

// ===== PT1 (1st-order IIR lowpass) =====

class PT1Filter {
private:
    float state;
    float k;
    bool initialized;
    bool enabled;

public:
    PT1Filter() : state(0.0f), k(1.0f), initialized(false), enabled(false) {}

    void init(float cutoffHz, float sampleRateHz, bool enable = true) {
        enabled = enable;
        state = 0.0f;
        initialized = false;

        if (!enabled || cutoffHz <= 0.0f || sampleRateHz <= 0.0f) {
            k = 1.0f;
            enabled = false;
            return;
        }

        float rc = 1.0f / (2.0f * PI * cutoffHz);
        float dt = 1.0f / sampleRateHz;
        k = dt / (rc + dt);
    }

    /** Retune cutoff at runtime without losing state. */
    void reconfigure(float cutoffHz, float sampleRateHz) {
        if (cutoffHz <= 0.0f || sampleRateHz <= 0.0f) {
            k = 1.0f;
            return;
        }
        float rc = 1.0f / (2.0f * PI * cutoffHz);
        float dt = 1.0f / sampleRateHz;
        k = dt / (rc + dt);
    }

    float update(float input) {
        if (!enabled) return input;

        if (!initialized) {
            state = input;
            initialized = true;
            return input;
        }

        state = state + k * (input - state);
        return state;
    }

    void reset() {
        state = 0.0f;
        initialized = false;
    }

    float getState() const { return state; }
    bool isEnabled() const { return enabled; }
    void setEnabled(bool en) { enabled = en; }
};

// ===== PT2 (2nd-order IIR lowpass) =====

class PT2Filter {
private:
    float state1, state2;
    float k;
    bool initialized;
    bool enabled;

public:
    PT2Filter() : state1(0.0f), state2(0.0f), k(1.0f), initialized(false), enabled(false) {}

    void init(float cutoffHz, float sampleRateHz, bool enable = true) {
        enabled = enable;
        state1 = state2 = 0.0f;
        initialized = false;

        if (!enabled || cutoffHz <= 0.0f || sampleRateHz <= 0.0f) {
            k = 1.0f;
            enabled = false;
            return;
        }

        float rc = 1.0f / (2.0f * PI * cutoffHz);
        float dt = 1.0f / sampleRateHz;
        k = dt / (rc + dt);
    }

    float update(float input) {
        if (!enabled) return input;

        if (!initialized) {
            state1 = state2 = input;
            initialized = true;
            return input;
        }

        state1 = state1 + k * (input - state1);
        state2 = state2 + k * (state1 - state2);
        return state2;
    }

    void reset() {
        state1 = state2 = 0.0f;
        initialized = false;
    }

    bool isEnabled() const { return enabled; }
    void setEnabled(bool en) { enabled = en; }
};

// ===== PT3 (3rd-order IIR lowpass) =====

class PT3Filter {
private:
    float state1, state2, state3;
    float k;
    bool initialized;
    bool enabled;

public:
    PT3Filter() : state1(0.0f), state2(0.0f), state3(0.0f), k(1.0f),
                  initialized(false), enabled(false) {}

    void init(float cutoffHz, float sampleRateHz, bool enable = true) {
        enabled = enable;
        state1 = state2 = state3 = 0.0f;
        initialized = false;

        if (!enabled || cutoffHz <= 0.0f || sampleRateHz <= 0.0f) {
            k = 1.0f;
            enabled = false;
            return;
        }

        float rc = 1.0f / (2.0f * PI * cutoffHz);
        float dt = 1.0f / sampleRateHz;
        k = dt / (rc + dt);
    }

    float update(float input) {
        if (!enabled) return input;

        if (!initialized) {
            state1 = state2 = state3 = input;
            initialized = true;
            return input;
        }

        state1 = state1 + k * (input - state1);
        state2 = state2 + k * (state1 - state2);
        state3 = state3 + k * (state2 - state3);
        return state3;
    }

    void reset() {
        state1 = state2 = state3 = 0.0f;
        initialized = false;
    }

    bool isEnabled() const { return enabled; }
    void setEnabled(bool en) { enabled = en; }
};

// ===== BIQUAD (2nd-order IIR, multi-type) =====

enum BiquadFilterType {
    BIQUAD_LOWPASS,
    BIQUAD_HIGHPASS,
    BIQUAD_BANDPASS,
    BIQUAD_NOTCH,
    BIQUAD_PEAK,
    BIQUAD_LOWSHELF,
    BIQUAD_HIGHSHELF
};

class BiquadFilter {
private:
    float b0, b1, b2, a1, a2;   // coefficients
    float x1, x2, y1, y2;       // state
    bool initialized;
    bool enabled;

public:
    BiquadFilter() : b0(1.0f), b1(0.0f), b2(0.0f), a1(0.0f), a2(0.0f),
                     x1(0.0f), x2(0.0f), y1(0.0f), y2(0.0f),
                     initialized(false), enabled(false) {}

    /** Q=0.707 gives Butterworth response. */
    void initLowpass(float cutoffHz, float sampleRateHz, float Q = 0.707f, bool enable = true) {
        enabled = enable;
        x1 = x2 = y1 = y2 = 0.0f;
        initialized = false;

        if (!enabled || cutoffHz <= 0.0f || sampleRateHz <= 0.0f) {
            b0 = 1.0f; b1 = 0.0f; b2 = 0.0f; a1 = 0.0f; a2 = 0.0f;
            enabled = false;
            return;
        }

        float omega = 2.0f * PI * cutoffHz / sampleRateHz;
        float sn = sinf(omega);
        float cs = cosf(omega);
        float alpha = sn / (2.0f * Q);

        float a0 = 1.0f + alpha;
        b0 = ((1.0f - cs) / 2.0f) / a0;
        b1 = (1.0f - cs) / a0;
        b2 = ((1.0f - cs) / 2.0f) / a0;
        a1 = (-2.0f * cs) / a0;
        a2 = (1.0f - alpha) / a0;
    }

    void initHighpass(float cutoffHz, float sampleRateHz, float Q = 0.707f, bool enable = true) {
        enabled = enable;
        x1 = x2 = y1 = y2 = 0.0f;
        initialized = false;

        if (!enabled || cutoffHz <= 0.0f || sampleRateHz <= 0.0f) {
            b0 = 1.0f; b1 = 0.0f; b2 = 0.0f; a1 = 0.0f; a2 = 0.0f;
            enabled = false;
            return;
        }

        float omega = 2.0f * PI * cutoffHz / sampleRateHz;
        float sn = sinf(omega);
        float cs = cosf(omega);
        float alpha = sn / (2.0f * Q);

        float a0 = 1.0f + alpha;
        b0 = ((1.0f + cs) / 2.0f) / a0;
        b1 = (-(1.0f + cs)) / a0;
        b2 = ((1.0f + cs) / 2.0f) / a0;
        a1 = (-2.0f * cs) / a0;
        a2 = (1.0f - alpha) / a0;
    }

    /** Higher Q narrows the notch. */
    void initNotch(float centerHz, float sampleRateHz, float Q = 5.0f, bool enable = true) {
        enabled = enable;
        x1 = x2 = y1 = y2 = 0.0f;
        initialized = false;

        if (!enabled || centerHz <= 0.0f || sampleRateHz <= 0.0f) {
            b0 = 1.0f; b1 = 0.0f; b2 = 0.0f; a1 = 0.0f; a2 = 0.0f;
            enabled = false;
            return;
        }

        float omega = 2.0f * PI * centerHz / sampleRateHz;
        float sn = sinf(omega);
        float cs = cosf(omega);
        float alpha = sn / (2.0f * Q);

        float a0 = 1.0f + alpha;
        b0 = 1.0f / a0;
        b1 = (-2.0f * cs) / a0;
        b2 = 1.0f / a0;
        a1 = (-2.0f * cs) / a0;
        a2 = (1.0f - alpha) / a0;
    }

    /** Retune cutoff at runtime for dynamic filtering. */
    void reconfigure(float cutoffHz, float sampleRateHz, float Q = 0.707f) {
        if (!enabled || cutoffHz <= 0.0f || sampleRateHz <= 0.0f) return;

        float omega = 2.0f * PI * cutoffHz / sampleRateHz;
        float sn = sinf(omega);
        float cs = cosf(omega);
        float alpha = sn / (2.0f * Q);

        float a0 = 1.0f + alpha;
        b0 = ((1.0f - cs) / 2.0f) / a0;
        b1 = (1.0f - cs) / a0;
        b2 = ((1.0f - cs) / 2.0f) / a0;
        a1 = (-2.0f * cs) / a0;
        a2 = (1.0f - alpha) / a0;
    }

    float update(float input) {
        if (!enabled) return input;

        if (!initialized) {
            x1 = x2 = input;
            y1 = y2 = input;
            initialized = true;
            return input;
        }

        float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = output;

        return output;
    }

    void reset() {
        x1 = x2 = y1 = y2 = 0.0f;
        initialized = false;
    }

    bool isEnabled() const { return enabled; }
    void setEnabled(bool en) { enabled = en; }
};

// ===== SLEW RATE LIMITER =====

class SlewFilter {
private:
    float state;
    float slewRateLimit;   // max change per sample
    bool initialized;
    bool enabled;

public:
    SlewFilter() : state(0.0f), slewRateLimit(1000.0f), initialized(false), enabled(false) {}

    void init(float maxRatePerSec, float sampleRateHz, bool enable = true) {
        enabled = enable;
        state = 0.0f;
        initialized = false;

        if (!enabled || maxRatePerSec <= 0.0f || sampleRateHz <= 0.0f) {
            slewRateLimit = 1e10f;
            enabled = false;
            return;
        }

        slewRateLimit = maxRatePerSec / sampleRateHz;
    }

    float update(float input) {
        if (!enabled) return input;

        if (!initialized) {
            state = input;
            initialized = true;
            return input;
        }

        float delta = input - state;
        delta = constrain(delta, -slewRateLimit, slewRateLimit);
        state += delta;

        return state;
    }

    void reset() {
        state = 0.0f;
        initialized = false;
    }

    bool isEnabled() const { return enabled; }
    void setEnabled(bool en) { enabled = en; }
};

// ===== GYRO FILTER BANK (3 axes, PT1) =====

class GyroFilterBank {
public:
    PT1Filter rollFilter;
    PT1Filter pitchFilter;
    PT1Filter yawFilter;

    bool enabled;
    float cutoffHz;
    float sampleRate;

    GyroFilterBank() : enabled(false), cutoffHz(100.0f), sampleRate(1000.0f) {}

    void init(float hz, float rate, bool enable = true) {
        cutoffHz = hz;
        sampleRate = rate;
        enabled = enable;

        rollFilter.reset();
        pitchFilter.reset();
        yawFilter.reset();

        rollFilter.init(hz, rate, enable);
        pitchFilter.init(hz, rate, enable);
        yawFilter.init(hz, rate, enable);
    }

    void update(float gxIn, float gyIn, float gzIn,
                float& gxOut, float& gyOut, float& gzOut) {
        if (!enabled) {
            gxOut = gxIn;
            gyOut = gyIn;
            gzOut = gzIn;
            return;
        }

        gxOut = rollFilter.update(gxIn);
        gyOut = pitchFilter.update(gyIn);
        gzOut = yawFilter.update(gzIn);
    }

    void reset() {
        rollFilter.reset();
        pitchFilter.reset();
        yawFilter.reset();
    }

    void setEnabled(bool en) {
        enabled = en;
        rollFilter.setEnabled(en);
        pitchFilter.setEnabled(en);
        yawFilter.setEnabled(en);
    }

    bool isEnabled() const { return enabled; }
};

#endif
