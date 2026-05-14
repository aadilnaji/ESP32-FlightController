/** ESP32 flight controller: cascaded 250Hz angle / 1000Hz rate loops, Madgwick fusion, IBUS RC, WiFi telemetry. */

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "MPU6050.h"
#include "HMC5883L.h"
#include "Madgwick.h"
#include "Filters.h"
#include "PID.h"
#include "IBUSReceiver.h"
#include "dashboard.h"

// ===== WIFI =====
const char* WIFI_SSID     = "ssid";
const char* WIFI_PASSWORD = "password";

// ===== PINS =====
const int ESC1_PIN = 25, ESC2_PIN = 26, ESC3_PIN = 27, ESC4_PIN = 14;
#define IBUS_RX_PIN 16
#define IBUS_TX_PIN -1
#define I2C_SDA 21
#define I2C_SCL 22
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// ===== ESC =====
const int ESC_MIN_US = 1000, ESC_MAX_US = 1940, ESC_PWM_FREQ_HZ = 490;
#define THROTTLE_OUTPUT_MIN 1000
#define THROTTLE_OUTPUT_MAX 1940
#define THROTTLE_DEADBAND 50
#define MOTOR_IDLE_THROTTLE 1050
#define MOTOR_IDLE_ENABLED true

// ===== IBUS CHANNELS =====
#define CH_THROTTLE 3
#define CH_YAW 4
#define CH_PITCH 2
#define CH_ROLL 1
#define CH_ARM_SWITCH 5
#define CH_MODE_SWITCH 6

// ===== ARMING =====
#define ARM_SWITCH_THRESHOLD 1700
#define DISARM_SWITCH_THRESHOLD 1300
#define THROTTLE_ARM_MAX 1050
#define RC_TIMEOUT_MS 500

// ===== TIMING =====
const uint32_t GYRO_LOOP_HZ = 1000;
const uint32_t GYRO_LOOP_US = 1000000 / GYRO_LOOP_HZ;
const uint32_t FUSION_DIVIDER = 4;
const uint32_t MAG_DIVIDER = 67;

// ===== CONTROL LIMITS =====
#define MAX_ANGLE_DEG 30.0f
#define MAX_RATE_DPS 500.0f

// ===== MADGWICK =====
#define MADGWICK_BETA_STARTUP 0.5f
#define MADGWICK_BETA_FLIGHT 0.1f

// ===== RATES =====
#define RC_RATE_ROLL 100
#define RC_RATE_PITCH 100
#define RC_RATE_YAW 100
#define SUPER_RATE_ROLL 70
#define SUPER_RATE_PITCH 70
#define SUPER_RATE_YAW 50
#define RC_EXPO_ROLL 20
#define RC_EXPO_PITCH 20
#define RC_EXPO_YAW 10
#define RATE_LIMIT_ROLL 500
#define RATE_LIMIT_PITCH 500
#define RATE_LIMIT_YAW 400

// ===== TPA =====
#define TPA_BREAKPOINT 1700
#define TPA_SCALE 15

// ===== FILTERS =====
#define GYRO_LOWPASS_ENABLED true
#define GYRO_LOWPASS_HZ 85.0f
#define DTERM_LOWPASS1_ENABLED true
#define DTERM_LOWPASS1_HZ 100.0f
#define DTERM_LOWPASS2_ENABLED false
#define DTERM_LOWPASS2_HZ 200.0f
#define DTERM_NOTCH_ENABLED false
#define DTERM_NOTCH_HZ 150.0f
#define DTERM_NOTCH_Q 5.0f
#define FTERM_FILTER_ENABLED true
#define FTERM_FILTER_HZ 100.0f
#define ITERM_RELAX_ENABLED false
#define ITERM_RELAX_HZ 15.0f
#define ITERM_RELAX_CUTOFF 40.0f
#define PTERM_FILTER_ENABLED false
#define PTERM_FILTER_HZ 100.0f

// ===== RATE PID =====
#define RATE_ROLL_KP 2.4f
#define RATE_ROLL_KI 1.6f
#define RATE_ROLL_KD 0.0006f
#define RATE_ROLL_KF 0.02f
#define RATE_ROLL_ILIMIT 60.0f
#define RATE_PITCH_KP 2.4f
#define RATE_PITCH_KI 1.6f
#define RATE_PITCH_KD 0.0006f
#define RATE_PITCH_KF 0.02f
#define RATE_PITCH_ILIMIT 60.0f
#define RATE_YAW_KP 1.3f
#define RATE_YAW_KI 1.6f
#define RATE_YAW_KD 0.0f
#define RATE_YAW_KF 0.01f
#define RATE_YAW_ILIMIT 80.0f

// ===== ANGLE PID =====
#define ANGLE_ROLL_KP 1.25f
#define ANGLE_ROLL_KI 0.5f
#define ANGLE_ROLL_KD 0.0f
#define ANGLE_ROLL_ILIMIT 40.0f
#define ANGLE_PITCH_KP 1.3f
#define ANGLE_PITCH_KI 0.5f
#define ANGLE_PITCH_KD 0.0f
#define ANGLE_PITCH_ILIMIT 40.0f

// ===== MAGNETOMETER CALIBRATION =====
// Flip CALIBRATE_MAG to true to recapture; serial output prints the new defines.
#define CALIBRATE_MAG   false

#define MAG_OFFSET_X  44.60f
#define MAG_OFFSET_Y  -192.40f
#define MAG_OFFSET_Z  -62.40f
#define MAG_SCALE_X   0.9775f
#define MAG_SCALE_Y   0.9881f
#define MAG_SCALE_Z   1.0364f

// ===== ACCELEROMETER CALIBRATION =====
// Flip CALIBRATE_ACCEL to true to recapture; serial output prints the new defines.
#define CALIBRATE_ACCEL false

#define ACCEL_OFFSET_X 0.036650f
#define ACCEL_OFFSET_Y 0.049102f
#define ACCEL_OFFSET_Z -0.102405f

// ===== LEVEL TRIM =====
// Positive roll = drone needs right roll to hover level; positive pitch = forward.
#define LEVEL_TRIM_ROLL_DEG   -1.2f
#define LEVEL_TRIM_PITCH_DEG  -1.2f

// ===== LOGGER =====
const size_t LOG_SAMPLES = 1500;
#define LOG_RAW_GYRO
#define LOG_FILTERED_GYRO
#define LOG_FUSED_ANGLES
#define LOG_PID_OUTPUTS
#define LOG_PID_COMMANDS
#define LOG_MOTOR_COMMANDS

enum FlightMode { MODE_ACRO = 0, MODE_STABILIZE = 1 };

struct TelemetryData {
    float roll, pitch, yaw;
    float angleErrRoll, angleErrPitch;

    float gyroX, gyroY, gyroZ;           // raw
    float gyroXf, gyroYf, gyroZf;        // filtered

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


typedef struct {
    uint32_t timestamp_us;
#ifdef LOG_RAW_GYRO
    int16_t gyro_x, gyro_y, gyro_z;
#endif
#ifdef LOG_FILTERED_GYRO
    int16_t gyro_xf, gyro_yf, gyro_zf;
#endif
#ifdef LOG_FUSED_ANGLES
    int16_t roll, pitch, yaw;
#endif
#ifdef LOG_PID_OUTPUTS
    int16_t pid_roll, pid_pitch, pid_yaw;
#endif
#ifdef LOG_PID_COMMANDS
    int16_t cmd_roll, cmd_pitch, cmd_yaw;
#endif
#ifdef LOG_MOTOR_COMMANDS
    uint16_t throttle, m1, m2, m3, m4;
#endif
} LoopLog;

// ===== GLOBALS =====
MadgwickFusion fusion(MADGWICK_BETA_STARTUP, MADGWICK_BETA_FLIGHT);
WebServer server(80);
Servo esc1, esc2, esc3, esc4;
IBUSReceiver ibus(Serial2, IBUS_RX_PIN, IBUS_TX_PIN);
GyroFilterBank gyroFilter;
PIDController rateRollPID, ratePitchPID, rateYawPID;
PIDController angleRollPID, anglePitchPID;
RateCurve rateCurve;
TPAController tpa;

portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool motorsArmed = false;
volatile FlightMode flightMode = MODE_STABILIZE;
volatile bool rcConnected = false;
volatile bool armSwitchEngaged = false;
volatile bool airmodeAllowed = false;
volatile int rcThrottle = ESC_MIN_US;
volatile float rcRoll = 0.0f, rcPitch = 0.0f, rcYaw = 0.0f;
volatile bool fusionConverged = false;
volatile float levelTrimRoll = LEVEL_TRIM_ROLL_DEG;
volatile float levelTrimPitch = LEVEL_TRIM_PITCH_DEG;

// ===== HEADLESS MODE =====
volatile bool headlessModeActive = false;
volatile float headlessReferenceYaw = 0.0f;    // yaw captured at activation
volatile bool headlessSwitchEngaged = false;   // switch edge-detect
volatile float currentFusedYaw = 0.0f;         // shared from flight task to IBUS task
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295f
#endif

TelemetryData telemetry;
static LoopLog logs[LOG_SAMPLES];
volatile bool loggingActive = false, loggingComplete = false;
volatile uint32_t logIndex = 0, loggingStartMs = 0, loggingDurationMs = 0;
volatile uint32_t loopTimeMin = UINT32_MAX;
volatile uint32_t loopTimeMax = 0;
volatile uint64_t loopTimeSum = 0;
volatile uint32_t loopCount = 0;
volatile uint32_t loopOverruns = 0;
volatile uint32_t lastLoopTime = 0;

volatile int rcArmSwitchVal = 1000;
volatile int rcModeSwitchVal = 1000;

static inline int16_t scaleToInt16(float val, float scale) {
    return (int16_t)constrain((int32_t)roundf(val * scale), -32768, 32767);
}

// ===== MOTOR FUNCTIONS =====
void mixAndWriteMotors(int thr, float roll, float pitch, float yaw, int &o1, int &o2, int &o3, int &o4) {
    float m1 = thr - pitch + roll - yaw;
    float m2 = thr + pitch + roll + yaw;
    float m3 = thr + pitch - roll - yaw;
    float m4 = thr - pitch - roll + yaw;
    o1 = constrain((int)roundf(m1), ESC_MIN_US, ESC_MAX_US);
    o2 = constrain((int)roundf(m2), ESC_MIN_US, ESC_MAX_US);
    o3 = constrain((int)roundf(m3), ESC_MIN_US, ESC_MAX_US);
    o4 = constrain((int)roundf(m4), ESC_MIN_US, ESC_MAX_US);
    esc1.writeMicroseconds(o1);
    esc2.writeMicroseconds(o2);
    esc3.writeMicroseconds(o3);
    esc4.writeMicroseconds(o4);
}

void writeMotorIdle() {
    int idle = MOTOR_IDLE_ENABLED ? MOTOR_IDLE_THROTTLE : ESC_MIN_US;
    esc1.writeMicroseconds(idle);
    esc2.writeMicroseconds(idle);
    esc3.writeMicroseconds(idle);
    esc4.writeMicroseconds(idle);
}

void stopMotors() {
    esc1.writeMicroseconds(ESC_MIN_US);
    esc2.writeMicroseconds(ESC_MIN_US);
    esc3.writeMicroseconds(ESC_MIN_US);
    esc4.writeMicroseconds(ESC_MIN_US);
}

void armEscs() { stopMotors(); delay(2000); }

// ===== HEADLESS MODE =====

/** Rotate roll/pitch stick from pilot/world frame back into the drone's body frame. */
void transformHeadlessInputs(float& roll, float& pitch, float currentYaw, float referenceYaw) {
    float deltaYaw = (currentYaw - referenceYaw) * DEG_TO_RAD;

    float cosYaw = cosf(deltaYaw);
    float sinYaw = sinf(deltaYaw);

    float newRoll = roll * cosYaw + pitch * sinYaw;
    float newPitch = -roll * sinYaw + pitch * cosYaw;

    roll = newRoll;
    pitch = newPitch;
}

// ===== LOGGER =====
void printLogHeader() {
    Serial.print("timestamp_us");
#ifdef LOG_RAW_GYRO
    Serial.print(",gx,gy,gz");
#endif
#ifdef LOG_FILTERED_GYRO
    Serial.print(",gxf,gyf,gzf");
#endif
#ifdef LOG_FUSED_ANGLES
    Serial.print(",roll,pitch,yaw");
#endif
#ifdef LOG_PID_OUTPUTS
    Serial.print(",pid_r,pid_p,pid_y");
#endif
#ifdef LOG_PID_COMMANDS
    Serial.print(",cmd_r,cmd_p,cmd_y");
#endif
#ifdef LOG_MOTOR_COMMANDS
    Serial.print(",thr,m1,m2,m3,m4");
#endif
    Serial.println();
}

void dumpLogData() {
    Serial.println("\n===== DATA DUMP =====");
    printLogHeader();
    for (uint32_t i = 0; i < logIndex; i++) {
        LoopLog &e = logs[i];
        Serial.print(e.timestamp_us);
#ifdef LOG_RAW_GYRO
        Serial.printf(",%.2f,%.2f,%.2f", e.gyro_x/100.0f, e.gyro_y/100.0f, e.gyro_z/100.0f);
#endif
#ifdef LOG_FILTERED_GYRO
        Serial.printf(",%.2f,%.2f,%.2f", e.gyro_xf/100.0f, e.gyro_yf/100.0f, e.gyro_zf/100.0f);
#endif
#ifdef LOG_FUSED_ANGLES
        Serial.printf(",%.2f,%.2f,%.2f", e.roll/100.0f, e.pitch/100.0f, e.yaw/100.0f);
#endif
#ifdef LOG_PID_OUTPUTS
        Serial.printf(",%.1f,%.1f,%.1f", e.pid_roll/10.0f, e.pid_pitch/10.0f, e.pid_yaw/10.0f);
#endif
#ifdef LOG_PID_COMMANDS
        Serial.printf(",%.1f,%.1f,%.1f", e.cmd_roll/10.0f, e.cmd_pitch/10.0f, e.cmd_yaw/10.0f);
#endif
#ifdef LOG_MOTOR_COMMANDS
        Serial.printf(",%u,%u,%u,%u,%u", e.throttle, e.m1, e.m2, e.m3, e.m4);
#endif
        Serial.println();
    }
    Serial.println("===== END =====\n");
}

void startLogging(int secs) {
    portENTER_CRITICAL(&stateMux);
    logIndex = 0;
    loggingActive = true;
    loggingComplete = false;
    loggingStartMs = millis();
    loggingDurationMs = (uint32_t)secs * 1000u;
    portEXIT_CRITICAL(&stateMux);
    Serial.printf("Logging for %ds\n", secs);
}

void stopLogging() {
    portENTER_CRITICAL(&stateMux);
    loggingActive = false;
    loggingComplete = true;
    portEXIT_CRITICAL(&stateMux);
    Serial.printf("Logged %d samples\n", logIndex);
}

// ===== INIT FUNCTIONS =====
void initFilters() {
    gyroFilter.init(GYRO_LOWPASS_HZ, (float)GYRO_LOOP_HZ, GYRO_LOWPASS_ENABLED);
    Serial.printf("Gyro LP: %s @ %.0fHz\n", GYRO_LOWPASS_ENABLED ? "ON" : "OFF", GYRO_LOWPASS_HZ);
}

void initPIDs() {
    float innerRate = (float)GYRO_LOOP_HZ;
    float outerRate = innerRate / FUSION_DIVIDER;
    
    rateRollPID.init(innerRate, PTERM_FILTER_HZ, PTERM_FILTER_ENABLED,
                     DTERM_NOTCH_HZ, DTERM_NOTCH_Q, DTERM_NOTCH_ENABLED,
                     DTERM_LOWPASS1_HZ, DTERM_LOWPASS1_ENABLED,
                     DTERM_LOWPASS2_HZ, DTERM_LOWPASS2_ENABLED,
                     FTERM_FILTER_HZ, FTERM_FILTER_ENABLED,
                     ITERM_RELAX_HZ, ITERM_RELAX_ENABLED, ITERM_RELAX_CUTOFF);
    rateRollPID.setGains(RATE_ROLL_KP, RATE_ROLL_KI, RATE_ROLL_KD, RATE_ROLL_KF);
    rateRollPID.setLimits(RATE_ROLL_ILIMIT, 500.0f);
    
    ratePitchPID.init(innerRate, PTERM_FILTER_HZ, PTERM_FILTER_ENABLED,
                      DTERM_NOTCH_HZ, DTERM_NOTCH_Q, DTERM_NOTCH_ENABLED,
                      DTERM_LOWPASS1_HZ, DTERM_LOWPASS1_ENABLED,
                      DTERM_LOWPASS2_HZ, DTERM_LOWPASS2_ENABLED,
                      FTERM_FILTER_HZ, FTERM_FILTER_ENABLED,
                      ITERM_RELAX_HZ, ITERM_RELAX_ENABLED, ITERM_RELAX_CUTOFF);
    ratePitchPID.setGains(RATE_PITCH_KP, RATE_PITCH_KI, RATE_PITCH_KD, RATE_PITCH_KF);
    ratePitchPID.setLimits(RATE_PITCH_ILIMIT, 500.0f);
    
    rateYawPID.init(innerRate, PTERM_FILTER_HZ, PTERM_FILTER_ENABLED,
                    DTERM_NOTCH_HZ, DTERM_NOTCH_Q, DTERM_NOTCH_ENABLED,
                    DTERM_LOWPASS1_HZ, DTERM_LOWPASS1_ENABLED,
                    DTERM_LOWPASS2_HZ, DTERM_LOWPASS2_ENABLED,
                    FTERM_FILTER_HZ, FTERM_FILTER_ENABLED,
                    ITERM_RELAX_HZ, ITERM_RELAX_ENABLED, ITERM_RELAX_CUTOFF);
    rateYawPID.setGains(RATE_YAW_KP, RATE_YAW_KI, RATE_YAW_KD, RATE_YAW_KF);
    rateYawPID.setLimits(RATE_YAW_ILIMIT, 500.0f);
    
    angleRollPID.init(outerRate, 50.0f, false, 0,0,false, 0,false, 0,false, 0,false, 0,false,0);
    angleRollPID.setGains(ANGLE_ROLL_KP, ANGLE_ROLL_KI, ANGLE_ROLL_KD, 0);
    angleRollPID.setLimits(ANGLE_ROLL_ILIMIT, MAX_RATE_DPS);
    
    anglePitchPID.init(outerRate, 50.0f, false, 0,0,false, 0,false, 0,false, 0,false, 0,false,0);
    anglePitchPID.setGains(ANGLE_PITCH_KP, ANGLE_PITCH_KI, ANGLE_PITCH_KD, 0);
    anglePitchPID.setLimits(ANGLE_PITCH_ILIMIT, MAX_RATE_DPS);
    
    rateRollPID.setFeedForwardScale(0);
    ratePitchPID.setFeedForwardScale(0);
    rateYawPID.setFeedForwardScale(0);
}

void initRateCurves() {
    rateCurve.setRates(0, RC_RATE_ROLL, SUPER_RATE_ROLL, RC_EXPO_ROLL, RATE_LIMIT_ROLL);
    rateCurve.setRates(1, RC_RATE_PITCH, SUPER_RATE_PITCH, RC_EXPO_PITCH, RATE_LIMIT_PITCH);
    rateCurve.setRates(2, RC_RATE_YAW, SUPER_RATE_YAW, RC_EXPO_YAW, RATE_LIMIT_YAW);
}

void initTPA() { tpa.setConfig(TPA_BREAKPOINT, TPA_SCALE); tpa.enabled = true; }

// ===== FLIGHT TASK =====
void flightTask(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(100));
    Serial.println("Flight task started");
    
    uint32_t fusionCounter = 0, magCounter = 0, convergenceCounter = 0;
    const uint32_t CONVERGENCE_LOOPS = 3000;
    uint64_t nextLoopTime = micros();
    
    float AccX, AccY, AccZ, GyroX, GyroY, GyroZ, GyroXf, GyroYf, GyroZf, TempC;
    float MagX = 0, MagY = 0, MagZ = 0;
    float fusedRoll = 0, fusedPitch = 0, fusedYaw = 0;
    bool fusionValid = false;
    float rateSetRoll = 0, rateSetPitch = 0, rateSetYaw = 0;
    FlightMode prevMode = MODE_STABILIZE;
    
    for (;;) {
        uint64_t now = micros();
        if ((int64_t)(nextLoopTime - now) > 0) while (micros() < nextLoopTime) asm("nop");
        nextLoopTime += GYRO_LOOP_US;
        uint64_t loopStart = micros();
        
        // 1. Read IMU
        if (!mpu6050_read(AccX, AccY, AccZ, GyroX, GyroY, GyroZ, TempC)) { stopMotors(); continue; }
        
        // 2. Filter gyro
        gyroFilter.update(GyroX, GyroY, GyroZ, GyroXf, GyroYf, GyroZf);
        
        // 2.5 Read mag
        bool magOk = false;
        if (++magCounter >= MAG_DIVIDER) { magCounter = 0; magOk = hmc5883l_read(MagX, MagY, MagZ); }
        
        // Get RC state
        float localRcRoll, localRcPitch, localRcYaw;
        int localThrottle;
        bool localArmed;
        FlightMode localMode;
        float trimR, trimP;
        portENTER_CRITICAL(&stateMux);
        localRcRoll = rcRoll; localRcPitch = rcPitch; localRcYaw = rcYaw;
        localThrottle = rcThrottle; localArmed = motorsArmed; localMode = flightMode;
        trimR = levelTrimRoll; trimP = levelTrimPitch;
        portEXIT_CRITICAL(&stateMux);
        
        // Mode change
        if (localMode != prevMode) {
            if (localMode == MODE_STABILIZE) {
                rateRollPID.setFeedForwardScale(0); ratePitchPID.setFeedForwardScale(0); rateYawPID.setFeedForwardScale(0);
            } else {
                rateRollPID.setFeedForwardScale(1); ratePitchPID.setFeedForwardScale(1); rateYawPID.setFeedForwardScale(1);
            }
            prevMode = localMode;
        }
        
        // 3. Fusion @ 250Hz
        if (++fusionCounter >= FUSION_DIVIDER) {
            fusionCounter = 0;
            MadgwickFusion::FusedOutput fused = fusion.update(AccX*9.81f, AccY*9.81f, AccZ*9.81f, GyroX, GyroY, GyroZ, MagX, MagY, MagZ, true, magOk);
            fusedRoll = fused.roll_deg; fusedPitch = fused.pitch_deg; fusedYaw = fused.yaw_deg; fusionValid = fused.data_valid;

            // Publish yaw so the IBUS task can capture it on headless activation
            portENTER_CRITICAL(&stateMux);
            currentFusedYaw = fusedYaw;
            portEXIT_CRITICAL(&stateMux);

            if (!fusionConverged && ++convergenceCounter >= CONVERGENCE_LOOPS) {
                fusion.setBeta(MADGWICK_BETA_FLIGHT);
                fusionConverged = true;
            }

            // 4. Outer loop (angle mode)
            if (localMode == MODE_STABILIZE && fusionValid) {
                float headlessRoll = localRcRoll;
                float headlessPitch = localRcPitch;
                bool localHeadless;
                float localRefYaw;
                portENTER_CRITICAL(&stateMux);
                localHeadless = headlessModeActive;
                localRefYaw = headlessReferenceYaw;
                portEXIT_CRITICAL(&stateMux);
                if (localHeadless) {
                    transformHeadlessInputs(headlessRoll, headlessPitch, fusedYaw, localRefYaw);
                }
                float targetRoll = headlessRoll * MAX_ANGLE_DEG + trimR;
                float targetPitch = headlessPitch * MAX_ANGLE_DEG + trimP;
                rateSetRoll = angleRollPID.update(targetRoll, fusedRoll);
                rateSetPitch = anglePitchPID.update(targetPitch, fusedPitch);
            }
        }

        // 4b. Acro mode (yaw always from rate curve)
        if (localMode == MODE_ACRO) {
            rateSetRoll = rateCurve.getRate(0, localRcRoll);
            rateSetPitch = rateCurve.getRate(1, localRcPitch);
        }
        rateSetYaw = rateCurve.getRate(2, localRcYaw);

        // 5. Inner rate loop
        float pidRoll = rateRollPID.update(rateSetRoll, GyroXf);
        float pidPitch = ratePitchPID.update(rateSetPitch, GyroYf);
        float pidYaw = rateYawPID.update(rateSetYaw, -GyroZf);

        // 6. TPA attenuation
        float tpaFactor = tpa.getFactor(localThrottle);
        float rollCmd = pidRoll * tpaFactor;
        float pitchCmd = pidPitch * tpaFactor;
        float yawCmd = pidYaw * tpaFactor;

        // 7. Motor output
        int out1, out2, out3, out4;
        if (localArmed) {
            bool shouldSpin = localThrottle > (ESC_MIN_US + THROTTLE_DEADBAND) || airmodeAllowed;
            if (shouldSpin) {
                mixAndWriteMotors(localThrottle, rollCmd, pitchCmd, yawCmd, out1, out2, out3, out4);
                if (!airmodeAllowed && localThrottle > 1400) { portENTER_CRITICAL(&stateMux); airmodeAllowed = true; portEXIT_CRITICAL(&stateMux); }
            } else {
                if (MOTOR_IDLE_ENABLED) { out1=out2=out3=out4=MOTOR_IDLE_THROTTLE; writeMotorIdle(); }
                else { out1=out2=out3=out4=ESC_MIN_US; stopMotors(); }
                rateRollPID.resetIterm(); ratePitchPID.resetIterm(); rateYawPID.resetIterm();
                angleRollPID.resetIterm(); anglePitchPID.resetIterm();
            }
        } else {
            out1=out2=out3=out4=ESC_MIN_US; stopMotors();
            rateRollPID.resetIterm(); ratePitchPID.resetIterm(); rateYawPID.resetIterm();
            angleRollPID.resetIterm(); anglePitchPID.resetIterm();
        }
        
        // Logging
        if (loggingActive && logIndex < LOG_SAMPLES) {
            LoopLog e; e.timestamp_us = (uint32_t)loopStart;
#ifdef LOG_RAW_GYRO
            e.gyro_x = scaleToInt16(GyroX, 100); e.gyro_y = scaleToInt16(GyroY, 100); e.gyro_z = scaleToInt16(GyroZ, 100);
#endif
#ifdef LOG_FILTERED_GYRO
            e.gyro_xf = scaleToInt16(GyroXf, 100); e.gyro_yf = scaleToInt16(GyroYf, 100); e.gyro_zf = scaleToInt16(GyroZf, 100);
#endif
#ifdef LOG_FUSED_ANGLES
            e.roll = scaleToInt16(fusedRoll, 100); e.pitch = scaleToInt16(fusedPitch, 100); e.yaw = scaleToInt16(fusedYaw, 100);
#endif
#ifdef LOG_PID_OUTPUTS
            e.pid_roll = scaleToInt16(pidRoll, 10); e.pid_pitch = scaleToInt16(pidPitch, 10); e.pid_yaw = scaleToInt16(pidYaw, 10);
#endif
#ifdef LOG_PID_COMMANDS
            e.cmd_roll = scaleToInt16(rollCmd, 10); e.cmd_pitch = scaleToInt16(pitchCmd, 10); e.cmd_yaw = scaleToInt16(yawCmd, 10);
#endif
#ifdef LOG_MOTOR_COMMANDS
            e.throttle = localThrottle; e.m1 = out1; e.m2 = out2; e.m3 = out3; e.m4 = out4;
#endif
            logs[logIndex++] = e;
            if (millis() - loggingStartMs >= loggingDurationMs) { loggingActive = false; loggingComplete = true; }
        }
        
        // Telemetry: rate errors and per-axis PID term breakdown
        float rateErrRoll = rateSetRoll - GyroXf;
        float rateErrPitch = rateSetPitch - GyroYf;
        float rateErrYaw = rateSetYaw - (-GyroZf);

        float pR = rateRollPID.getLastPTerm();
        float iR = rateRollPID.getLastITerm();
        float dR = rateRollPID.getLastDTerm();
        float fR = rateRollPID.getLastFTerm();

        float pP = ratePitchPID.getLastPTerm();
        float iP = ratePitchPID.getLastITerm();
        float dP = ratePitchPID.getLastDTerm();
        float fP = ratePitchPID.getLastFTerm();

        float pY = rateYawPID.getLastPTerm();
        float iY = rateYawPID.getLastITerm();
        float dY = rateYawPID.getLastDTerm();
        float fY = rateYawPID.getLastFTerm();

        float angleErrRoll = 0, angleErrPitch = 0;
        if (localMode == MODE_STABILIZE) {
            angleErrRoll = (localRcRoll * MAX_ANGLE_DEG + trimR) - fusedRoll;
            angleErrPitch = (localRcPitch * MAX_ANGLE_DEG + trimP) - fusedPitch;
        }

        bool saturated = (out1 >= ESC_MAX_US - 10 || out1 <= ESC_MIN_US + 10 ||
                        out2 >= ESC_MAX_US - 10 || out2 <= ESC_MIN_US + 10 ||
                        out3 >= ESC_MAX_US - 10 || out3 <= ESC_MIN_US + 10 ||
                        out4 >= ESC_MAX_US - 10 || out4 <= ESC_MIN_US + 10);

        uint32_t loopTime = micros() - loopStart;
        if (loopTime < loopTimeMin) loopTimeMin = loopTime;
        if (loopTime > loopTimeMax) loopTimeMax = loopTime;
        loopTimeSum += loopTime;
        loopCount++;
        if (loopTime > GYRO_LOOP_US + 100) loopOverruns++;
        uint32_t jitter = (loopTime > lastLoopTime) ? loopTime - lastLoopTime : lastLoopTime - loopTime;
        lastLoopTime = loopTime;

        // Publish telemetry snapshot at 50Hz (every 20 loops)
        static uint8_t telemCounter = 0;
        if (++telemCounter >= 20) {
            telemCounter = 0;
            portENTER_CRITICAL(&telemetryMux);

            telemetry.roll = fusedRoll;
            telemetry.pitch = fusedPitch;
            telemetry.yaw = fusedYaw;
            telemetry.angleErrRoll = angleErrRoll;
            telemetry.angleErrPitch = angleErrPitch;

            telemetry.gyroX = GyroX;
            telemetry.gyroY = GyroY;
            telemetry.gyroZ = GyroZ;
            telemetry.gyroXf = GyroXf;
            telemetry.gyroYf = GyroYf;
            telemetry.gyroZf = GyroZf;

            telemetry.accelX = AccX * 9.81f;
            telemetry.accelY = AccY * 9.81f;
            telemetry.accelZ = AccZ * 9.81f;
            telemetry.magX = MagX;
            telemetry.magY = MagY;
            telemetry.magZ = MagZ;
            telemetry.magValid = magOk;

            telemetry.rateSetRoll = rateSetRoll;
            telemetry.rateSetPitch = rateSetPitch;
            telemetry.rateSetYaw = rateSetYaw;
            telemetry.rateErrRoll = rateErrRoll;
            telemetry.rateErrPitch = rateErrPitch;
            telemetry.rateErrYaw = rateErrYaw;

            telemetry.pTermRoll = pR;
            telemetry.pTermPitch = pP;
            telemetry.pTermYaw = pY;
            telemetry.iTermRoll = iR;
            telemetry.iTermPitch = iP;
            telemetry.iTermYaw = iY;
            telemetry.dTermRoll = dR;
            telemetry.dTermPitch = dP;
            telemetry.dTermYaw = dY;
            telemetry.fTermRoll = fR;
            telemetry.fTermPitch = fP;
            telemetry.fTermYaw = fY;
            telemetry.pidTotalRoll = rollCmd;
            telemetry.pidTotalPitch = pitchCmd;
            telemetry.pidTotalYaw = yawCmd;

            telemetry.mixRoll = rollCmd;
            telemetry.mixPitch = pitchCmd;
            telemetry.mixYaw = yawCmd;
            telemetry.throttle = localThrottle;
            telemetry.m1 = out1;
            telemetry.m2 = out2;
            telemetry.m3 = out3;
            telemetry.m4 = out4;
            telemetry.motorSaturated = saturated;

            telemetry.rcRoll = localRcRoll;
            telemetry.rcPitch = localRcPitch;
            telemetry.rcYaw = localRcYaw;
            telemetry.rcThrottle = localThrottle;

            telemetry.armed = localArmed;
            telemetry.mode = localMode;
            telemetry.fusionConverged = fusionConverged;
            telemetry.airmodeActive = airmodeAllowed;
            telemetry.tpaFactor = tpaFactor;

            telemetry.trimRoll = trimR;
            telemetry.trimPitch = trimP;

            telemetry.headlessActive = headlessModeActive;
            telemetry.headlessRefYaw = headlessReferenceYaw;

            telemetry.loopTime = loopTime;
            telemetry.loopMin = loopTimeMin;
            telemetry.loopMax = loopTimeMax;
            telemetry.loopAvg = loopCount > 0 ? (uint32_t)(loopTimeSum / loopCount) : 0;
            telemetry.loopJitter = jitter;
            telemetry.overruns = loopOverruns;
            
            portEXIT_CRITICAL(&telemetryMux);
        }
    }
}

// ===== WEB SERVER =====
void setupWebServer() {
    server.on("/", HTTP_GET, []() { server.send(200, "text/html", HTML_PAGE); });
    
    server.on("/pid", HTTP_GET, []() {
        String axis = server.arg("axis");
        float kp = server.arg("kp").toFloat(), ki = server.arg("ki").toFloat();
        float kd = server.arg("kd").toFloat(), kf = server.hasArg("kf") ? server.arg("kf").toFloat() : 0;
        if (axis == "rr") { rateRollPID.setGains(kp, ki, kd, kf); rateRollPID.resetIterm(); }
        else if (axis == "rp") { ratePitchPID.setGains(kp, ki, kd, kf); ratePitchPID.resetIterm(); }
        else if (axis == "ry") { rateYawPID.setGains(kp, ki, kd, kf); rateYawPID.resetIterm(); }
        else if (axis == "ar") { angleRollPID.setGains(kp, ki, kd, 0); angleRollPID.resetIterm(); }
        else if (axis == "ap") { anglePitchPID.setGains(kp, ki, kd, 0); anglePitchPID.resetIterm(); }
        server.send(200, "text/plain", "OK");
    });
    
    server.on("/gyrofilter", HTTP_GET, []() {
        float hz = server.arg("hz").toFloat();
        bool en = server.hasArg("en") ? server.arg("en").toInt() > 0 : (hz > 0);
        gyroFilter.init(hz, (float)GYRO_LOOP_HZ, en);
        server.send(200, "text/plain", "OK");
    });
    
    server.on("/rates", HTTP_GET, []() {
        int axis = server.arg("axis").toInt();
        rateCurve.setRates(axis, server.arg("rc").toFloat(), server.arg("super").toFloat(),
                          server.arg("expo").toFloat(), server.arg("limit").toFloat());
        server.send(200, "text/plain", "OK");
    });
    
    server.on("/getpid", HTTP_GET, []() {
        String json = "{\"rr\":{\"kp\":" + String(rateRollPID.kp,3) + ",\"ki\":" + String(rateRollPID.ki,4) +
                      ",\"kd\":" + String(rateRollPID.kd,4) + ",\"kf\":" + String(rateRollPID.kf,4) + "},";
        json += "\"rp\":{\"kp\":" + String(ratePitchPID.kp,3) + ",\"ki\":" + String(ratePitchPID.ki,4) +
                ",\"kd\":" + String(ratePitchPID.kd,4) + ",\"kf\":" + String(ratePitchPID.kf,4) + "},";
        json += "\"ry\":{\"kp\":" + String(rateYawPID.kp,3) + ",\"ki\":" + String(rateYawPID.ki,4) +
                ",\"kd\":" + String(rateYawPID.kd,4) + ",\"kf\":" + String(rateYawPID.kf,4) + "},";
        json += "\"ar\":{\"kp\":" + String(angleRollPID.kp,3) + ",\"ki\":" + String(angleRollPID.ki,4) +
                ",\"kd\":" + String(angleRollPID.kd,4) + "},";
        json += "\"ap\":{\"kp\":" + String(anglePitchPID.kp,3) + ",\"ki\":" + String(anglePitchPID.ki,4) +
                ",\"kd\":" + String(anglePitchPID.kd,4) + "}}";
        server.send(200, "application/json", json);
    });
    
    server.on("/telem", HTTP_GET, []() {
        TelemetryData s;
        portENTER_CRITICAL(&telemetryMux);
        s = telemetry;
        portEXIT_CRITICAL(&telemetryMux);
        
        bool armed, rc;
        FlightMode mode;
        int armSw = 1000, modeSw = 1000;
        
        portENTER_CRITICAL(&stateMux);
        armed = motorsArmed;
        mode = flightMode;
        rc = rcConnected;
        armSw = rcArmSwitchVal;
        modeSw = rcModeSwitchVal;
        portEXIT_CRITICAL(&stateMux);
        
        // Compact JSON: short keys cut bandwidth on the ESP32 web server
        String json = "{";

        json += "\"r\":" + String(s.roll, 1);
        json += ",\"p\":" + String(s.pitch, 1);
        json += ",\"y\":" + String(s.yaw, 1);
        json += ",\"ae\":[" + String(s.angleErrRoll, 1) + "," + String(s.angleErrPitch, 1) + "]";

        json += ",\"rc\":[" + String(s.rcRoll, 3) + "," + String(s.rcPitch, 3) + "," +
                String(s.rcYaw, 3) + "," + String(s.rcThrottle) + "]";
        json += ",\"rcs\":[" + String(armSw) + "," + String(modeSw) + "]";

        json += ",\"gr\":[" + String(s.gyroX, 1) + "," + String(s.gyroY, 1) + "," + String(s.gyroZ, 1) + "]";
        json += ",\"gf\":[" + String(s.gyroXf, 1) + "," + String(s.gyroYf, 1) + "," + String(s.gyroZf, 1) + "]";

        json += ",\"rs\":[" + String(s.rateSetRoll, 1) + "," + String(s.rateSetPitch, 1) + "," + String(s.rateSetYaw, 1) + "]";
        json += ",\"re\":[" + String(s.rateErrRoll, 1) + "," + String(s.rateErrPitch, 1) + "," + String(s.rateErrYaw, 1) + "]";

        json += ",\"pp\":[" + String(s.pTermRoll, 0) + "," + String(s.pTermPitch, 0) + "," + String(s.pTermYaw, 0) + "]";
        json += ",\"pi\":[" + String(s.iTermRoll, 0) + "," + String(s.iTermPitch, 0) + "," + String(s.iTermYaw, 0) + "]";
        json += ",\"pd\":[" + String(s.dTermRoll, 0) + "," + String(s.dTermPitch, 0) + "," + String(s.dTermYaw, 0) + "]";
        json += ",\"pf\":[" + String(s.fTermRoll, 0) + "," + String(s.fTermPitch, 0) + "," + String(s.fTermYaw, 0) + "]";
        json += ",\"pt\":[" + String(s.pidTotalRoll, 1) + "," + String(s.pidTotalPitch, 1) + "," + String(s.pidTotalYaw, 1) + "]";

        json += ",\"thr\":" + String(s.throttle);
        json += ",\"m1\":" + String(s.m1);
        json += ",\"m2\":" + String(s.m2);
        json += ",\"m3\":" + String(s.m3);
        json += ",\"m4\":" + String(s.m4);
        json += ",\"mix\":[" + String(s.mixRoll, 1) + "," + String(s.mixPitch, 1) + "," + String(s.mixYaw, 1) + "]";
        json += ",\"sat\":" + String(s.motorSaturated ? "\"SAT\"" : "\"None\"");

        json += ",\"acc\":[" + String(s.accelX, 2) + "," + String(s.accelY, 2) + "," + String(s.accelZ, 2) + "]";
        json += ",\"mag\":[" + String(s.magX, 1) + "," + String(s.magY, 1) + "," + String(s.magZ, 1) + "]";

        json += ",\"armed\":" + String(armed ? "true" : "false");
        json += ",\"mode\":" + String((int)mode);
        json += ",\"rcOk\":" + String(rc ? "true" : "false");
        json += ",\"conv\":" + String(s.fusionConverged ? "true" : "false");
        json += ",\"accOk\":true";
        json += ",\"magOk\":" + String(s.magValid ? "true" : "false");
        json += ",\"air\":" + String(s.airmodeActive ? "true" : "false");
        json += ",\"gfOn\":" + String(gyroFilter.isEnabled() ? "true" : "false");

        json += ",\"tpa\":" + String(s.tpaFactor, 2);
        json += ",\"trim\":[" + String(s.trimRoll, 2) + "," + String(s.trimPitch, 2) + "]";

        json += ",\"hl\":" + String(s.headlessActive ? "true" : "false");
        json += ",\"hlRef\":" + String(s.headlessRefYaw, 1);

        json += ",\"lp\":" + String(s.loopTime);
        json += ",\"ls\":[" + String(s.loopTime) + "," + String(s.loopMin) + "," +
                String(s.loopMax) + "," + String(s.loopAvg) + "," +
                String(s.loopJitter) + "," + String(s.overruns) + "]";
        json += ",\"up\":" + String(millis() / 1000);

        json += "}";
        
        server.send(200, "application/json", json);
    });
    
    server.on("/anglepid", HTTP_GET, []() {
        String axis = server.arg("axis");
        float kp = server.arg("kp").toFloat();
        float ki = server.arg("ki").toFloat();
        float kd = server.arg("kd").toFloat();
        
        if (axis == "roll") {
            angleRollPID.setGains(kp, ki, kd, 0);
            angleRollPID.resetIterm();
        } else if (axis == "pitch") {
            anglePitchPID.setGains(kp, ki, kd, 0);
            anglePitchPID.resetIterm();
        }
        server.send(200, "text/plain", "OK");
    });

    server.on("/ratepid", HTTP_GET, []() {
        String axis = server.arg("axis");
        float kp = server.arg("kp").toFloat();
        float ki = server.arg("ki").toFloat();
        float kd = server.arg("kd").toFloat();
        float kf = server.hasArg("kf") ? server.arg("kf").toFloat() : 0;
        
        if (axis == "roll") {
            rateRollPID.setGains(kp, ki, kd, kf);
            rateRollPID.resetIterm();
        } else if (axis == "pitch") {
            ratePitchPID.setGains(kp, ki, kd, kf);
            ratePitchPID.resetIterm();
        } else if (axis == "yaw") {
            rateYawPID.setGains(kp, ki, kd, kf);
            rateYawPID.resetIterm();
        }
        server.send(200, "text/plain", "OK");
    });

    server.on("/madgwick", HTTP_GET, []() {
        if (server.hasArg("reset")) {
            fusion.reset();
            server.send(200, "text/plain", "Reset");
            return;
        }
        
        float beta = server.arg("beta").toFloat();
        if (beta > 0.0f) {
            fusion.setBeta(beta);
        }
        
        server.send(200, "text/plain", "OK");
    });

    server.on("/magcal", HTTP_GET, []() {
        Serial.println("Starting magnetometer calibration...");
        server.send(200, "text/plain", "Calibrating");
    });

    server.on("/filters", HTTP_GET, []() {
        bool gfEn = server.arg("gf_en").toInt() > 0;
        float gfHz = server.arg("gf_hz").toFloat();
        
        gyroFilter.init(gfHz, (float)GYRO_LOOP_HZ, gfEn);
        
        server.send(200, "text/plain", "OK");
    });

    server.on("/maxangle", HTTP_GET, []() {
        float deg = server.arg("deg").toFloat();
        (void)deg;  // stubbed: MAX_ANGLE_DEG would need to become non-const
        server.send(200, "text/plain", "OK");
    });

    server.on("/resetstats", HTTP_GET, []() {
        loopTimeMin = UINT32_MAX;
        loopTimeMax = 0;
        loopTimeSum = 0;
        loopCount = 0;
        loopOverruns = 0;
        server.send(200, "text/plain", "OK");
    });

    server.on("/estop", HTTP_GET, []() {
        portENTER_CRITICAL(&stateMux);
        motorsArmed = false;
        airmodeAllowed = false;
        portEXIT_CRITICAL(&stateMux);
        stopMotors();
        server.send(200, "text/plain", "STOPPED");
    });

    server.on("/resetpids", HTTP_GET, []() {
        initPIDs();
        server.send(200, "text/plain", "OK");
    });

    server.on("/trim", HTTP_GET, []() {
        if (server.hasArg("roll") || server.hasArg("pitch")) {
            portENTER_CRITICAL(&stateMux);
            if (server.hasArg("roll")) {
                levelTrimRoll = constrain(server.arg("roll").toFloat(), -10.0f, 10.0f);
            }
            if (server.hasArg("pitch")) {
                levelTrimPitch = constrain(server.arg("pitch").toFloat(), -10.0f, 10.0f);
            }
            portEXIT_CRITICAL(&stateMux);

            Serial.printf("Trim updated: Roll=%.2f° Pitch=%.2f°\n", levelTrimRoll, levelTrimPitch);
            server.send(200, "text/plain", "OK");
        } else {
            float r, p;
            portENTER_CRITICAL(&stateMux);
            r = levelTrimRoll;
            p = levelTrimPitch;
            portEXIT_CRITICAL(&stateMux);
            String json = "{\"roll\":" + String(r, 2) + ",\"pitch\":" + String(p, 2) + "}";
            server.send(200, "application/json", json);
        }
    });

    server.on("/calaccel", HTTP_GET, []() {
        if (server.hasArg("confirm") && server.arg("confirm") == "yes") {
            Serial.println("\n========================================");
            Serial.println("ACCELEROMETER CALIBRATION (via Web)");
            Serial.println("========================================");
            Serial.println("Calibrating... keep drone LEVEL and STATIONARY!");
            
            mpu6050_calibrate_accel();
            
            IMUCalibration cal = mpu6050_get_calibration();
            
            Serial.println("\nCalibration complete! Copy these values to main.cpp:");
            Serial.println("----------------------------------------");
            Serial.printf("#define ACCEL_OFFSET_X  %.6ff\n", cal.accelOffsetX);
            Serial.printf("#define ACCEL_OFFSET_Y  %.6ff\n", cal.accelOffsetY);
            Serial.printf("#define ACCEL_OFFSET_Z  %.6ff\n", cal.accelOffsetZ);
            Serial.println("----------------------------------------\n");
            
            String json = "{\"x\":" + String(cal.accelOffsetX, 6) + 
                          ",\"y\":" + String(cal.accelOffsetY, 6) + 
                          ",\"z\":" + String(cal.accelOffsetZ, 6) + "}";
            server.send(200, "application/json", json);
        } else {
            server.send(400, "text/plain", "Add ?confirm=yes to run calibration");
        }
    });

    server.on("/headless", HTTP_GET, []() {
        if (server.hasArg("toggle")) {
            portENTER_CRITICAL(&stateMux);
            if (!headlessModeActive) {
                headlessReferenceYaw = currentFusedYaw;  // capture heading on activate
                headlessModeActive = true;
                headlessSwitchEngaged = true;
            } else {
                headlessModeActive = false;
                headlessSwitchEngaged = false;
            }
            portEXIT_CRITICAL(&stateMux);
        }
        
        bool active;
        float refYaw;
        portENTER_CRITICAL(&stateMux);
        active = headlessModeActive;
        refYaw = headlessReferenceYaw;
        portEXIT_CRITICAL(&stateMux);
        
        String json = "{\"active\":" + String(active ? "true" : "false") +
                      ",\"refYaw\":" + String(refYaw, 1) + "}";
        server.send(200, "application/json", json);
    });

    server.begin();
}

// ===== SERIAL COMMANDS =====
void parseSerialLine(String &line) {
    line.trim(); if (line.length() == 0) return;
    String s = line; s.toUpperCase();
    
    if (s.startsWith("L")) { startLogging(s.substring(1).toInt() > 0 ? s.substring(1).toInt() : 5); return; }
    if (s == "S") { stopLogging(); return; }
    if (s == "D") { if (loggingComplete) dumpLogData(); return; }
    if (s == "ARM") { portENTER_CRITICAL(&stateMux); motorsArmed = true; portEXIT_CRITICAL(&stateMux); Serial.println("ARMED"); return; }
    if (s == "DISARM") { portENTER_CRITICAL(&stateMux); motorsArmed = false; airmodeAllowed = false; portEXIT_CRITICAL(&stateMux); stopMotors(); Serial.println("DISARMED"); return; }
    if (s == "ACRO") { portENTER_CRITICAL(&stateMux); flightMode = MODE_ACRO; portEXIT_CRITICAL(&stateMux); return; }
    if (s == "STAB") { portENTER_CRITICAL(&stateMux); flightMode = MODE_STABILIZE; portEXIT_CRITICAL(&stateMux); return; }
    if (s.startsWith("GF")) {
        if (s == "GFOFF") gyroFilter.setEnabled(false);
        else if (s == "GFON") gyroFilter.setEnabled(true);
        else { float hz = s.substring(2).toFloat(); if (hz > 0) gyroFilter.init(hz, GYRO_LOOP_HZ, true); }
        return;
    }
    if (s.startsWith("TRIMR")) {
        float val = s.substring(5).toFloat();
        portENTER_CRITICAL(&stateMux);
        levelTrimRoll = constrain(val, -10.0f, 10.0f);
        portEXIT_CRITICAL(&stateMux);
        Serial.printf("Roll trim: %.2f°\n", levelTrimRoll);
        return;
    }
    if (s.startsWith("TRIMP")) {
        float val = s.substring(5).toFloat();
        portENTER_CRITICAL(&stateMux);
        levelTrimPitch = constrain(val, -10.0f, 10.0f);
        portEXIT_CRITICAL(&stateMux);
        Serial.printf("Pitch trim: %.2f°\n", levelTrimPitch);
        return;
    }
    if (s == "TRIM") {
        Serial.printf("Current trim: Roll=%.2f° Pitch=%.2f°\n", levelTrimRoll, levelTrimPitch);
        return;
    }
    if (s == "STATUS") {
        TelemetryData snap; portENTER_CRITICAL(&telemetryMux); snap = telemetry; portEXIT_CRITICAL(&telemetryMux);
        Serial.printf("R=%.1f P=%.1f Y=%.1f | M:%d %d %d %d | Loop:%dus | Trim: R=%.1f P=%.1f\n", 
                      snap.roll, snap.pitch, snap.yaw, snap.m1, snap.m2, snap.m3, snap.m4, snap.loopTime,
                      snap.trimRoll, snap.trimPitch);
        return;
    }
    if (s == "PIDS") {
        Serial.printf("Roll: P=%.3f I=%.4f D=%.4f F=%.4f\n", rateRollPID.kp, rateRollPID.ki, rateRollPID.kd, rateRollPID.kf);
        Serial.printf("Pitch: P=%.3f I=%.4f D=%.4f F=%.4f\n", ratePitchPID.kp, ratePitchPID.ki, ratePitchPID.kd, ratePitchPID.kf);
        return;
    }
    // Manual control: token format "T1200 R0.5 P-0.3 Y0.1"
    int idx = 0;
    while (idx < (int)s.length()) {
        while (idx < (int)s.length() && isspace(s[idx])) idx++;
        if (idx >= (int)s.length()) break;
        int end = idx;
        while (end < (int)s.length() && s[end] != ' ' && s[end] != ',') end++;
        String token = s.substring(idx, end); idx = end + 1;
        if (token.length() < 2) continue;
        char c = token.charAt(0); String val = token.substring(1);
        portENTER_CRITICAL(&stateMux);
        if (c == 'T') rcThrottle = constrain(val.toInt(), ESC_MIN_US, ESC_MAX_US);
        else if (c == 'R') rcRoll = constrain(val.toFloat(), -1.0f, 1.0f);
        else if (c == 'P') rcPitch = constrain(val.toFloat(), -1.0f, 1.0f);
        else if (c == 'Y') rcYaw = constrain(val.toFloat(), -1.0f, 1.0f);
        portEXIT_CRITICAL(&stateMux);
    }
}

void serviceSerial() {
    static String line = "";
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') { if (line.length() > 0) { parseSerialLine(line); line = ""; } }
        else { line += c; if (line.length() > 200) line = line.substring(line.length() - 200); }
    }
}

// ===== IBUS TASK =====
void ibusTask(void *pvParameters) {
    bool wasConnected = false;
    for (;;) {
        bool newFrame = ibus.update();
        bool connected = ibus.isValid();
        
        if (connected != wasConnected) {
            if (!connected) {
                portENTER_CRITICAL(&stateMux);
                motorsArmed = false; armSwitchEngaged = false; rcConnected = false; airmodeAllowed = false;
                portEXIT_CRITICAL(&stateMux);
                stopMotors();
            }
            wasConnected = connected;
        }
        portENTER_CRITICAL(&stateMux); rcConnected = connected; portEXIT_CRITICAL(&stateMux);
        
        if (connected && newFrame) {
            uint16_t thr = map(ibus.getChannel(CH_THROTTLE), IBUS_MIN, IBUS_MAX, THROTTLE_OUTPUT_MIN, THROTTLE_OUTPUT_MAX);
            float roll = ibus.getChannelNormalized(CH_ROLL);
            float pitch = ibus.getChannelNormalized(CH_PITCH);
            float yaw = ibus.getChannelNormalized(CH_YAW);
            uint16_t armSw = ibus.getChannel(CH_ARM_SWITCH);
            uint16_t modeSw = ibus.getChannel(CH_MODE_SWITCH);
            
            bool armHigh = armSw > ARM_SWITCH_THRESHOLD;
            bool thrLow = thr < THROTTLE_ARM_MAX;
            
            portENTER_CRITICAL(&stateMux);
            bool armed = motorsArmed, engaged = armSwitchEngaged;
            portEXIT_CRITICAL(&stateMux);
            
            if (armHigh && thrLow && !engaged) {
                portENTER_CRITICAL(&stateMux); motorsArmed = true; armSwitchEngaged = true; portEXIT_CRITICAL(&stateMux);
            } else if (!armHigh && armSw < DISARM_SWITCH_THRESHOLD) {
                portENTER_CRITICAL(&stateMux); motorsArmed = false; armSwitchEngaged = false; airmodeAllowed = false; portEXIT_CRITICAL(&stateMux);
                stopMotors();
            }
            
            portENTER_CRITICAL(&stateMux);
            rcThrottle = thr; rcRoll = roll; rcPitch = pitch; rcYaw = yaw;
            rcArmSwitchVal = armSw; rcModeSwitchVal = modeSw;

            // CH6 currently drives headless toggle instead of mode switch (acro disabled while testing).
            bool headlessHigh = modeSw > 1700;
            bool wasHeadlessEngaged = headlessSwitchEngaged;
            float captureYaw = currentFusedYaw;

            if (headlessHigh && !wasHeadlessEngaged) {
                headlessReferenceYaw = captureYaw;
                headlessModeActive = true;
                headlessSwitchEngaged = true;
            } else if (!headlessHigh && modeSw < 1300) {
                headlessModeActive = false;
                headlessSwitchEngaged = false;
            }

            flightMode = MODE_STABILIZE;
            portEXIT_CRITICAL(&stateMux);
        }
        
        if (!connected && ibus.timeSinceLastFrame() > RC_TIMEOUT_MS) {
            portENTER_CRITICAL(&stateMux);
            if (motorsArmed) { motorsArmed = false; armSwitchEngaged = false; airmodeAllowed = false; }
            portEXIT_CRITICAL(&stateMux);
            stopMotors();
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ===== COMMS TASK =====
void commsTask(void *pvParameters) {
    for (;;) { server.handleClient(); serviceSerial(); vTaskDelay(pdMS_TO_TICKS(10)); }
}

// ===== SETUP =====
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== ESP32-FC ===\n");
    
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    // ===== ESCs =====
    Serial.println("Init ESCs...");
    esc1.setPeriodHertz(ESC_PWM_FREQ_HZ); esc2.setPeriodHertz(ESC_PWM_FREQ_HZ);
    esc3.setPeriodHertz(ESC_PWM_FREQ_HZ); esc4.setPeriodHertz(ESC_PWM_FREQ_HZ);
    esc1.attach(ESC1_PIN, ESC_MIN_US, ESC_MAX_US); esc2.attach(ESC2_PIN, ESC_MIN_US, ESC_MAX_US);
    esc3.attach(ESC3_PIN, ESC_MIN_US, ESC_MAX_US); esc4.attach(ESC4_PIN, ESC_MIN_US, ESC_MAX_US);
    armEscs();
    delay(4000);
    
    // ===== MPU6050 =====
    Serial.println("Init MPU6050...");
    mpu6050_init();

#if CALIBRATE_ACCEL
    // Calibration mode: drone must be on a perfectly level surface. Copy printed
    // values into ACCEL_OFFSET_X/Y/Z, then flip CALIBRATE_ACCEL back to false.
    Serial.println("\n=== ACCELEROMETER CALIBRATION ===");
    Serial.println("Place drone on a PERFECTLY LEVEL surface!");
    Serial.println("Starting in 3 seconds...");
    delay(3000);

    digitalWrite(LED_BUILTIN, HIGH);
    mpu6050_calibrate_accel();
    digitalWrite(LED_BUILTIN, LOW);

    IMUCalibration cal = mpu6050_get_calibration();

    Serial.println("\n=== COMPLETE - copy to main.cpp ===");
    Serial.printf("#define ACCEL_OFFSET_X  %.6ff\n", cal.accelOffsetX);
    Serial.printf("#define ACCEL_OFFSET_Y  %.6ff\n", cal.accelOffsetY);
    Serial.printf("#define ACCEL_OFFSET_Z  %.6ff\n", cal.accelOffsetZ);
    Serial.println("Then set CALIBRATE_ACCEL=false and re-upload.");

    for (int i = 0; i < 10; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }

    while (true) {
        delay(1000);
    }
#else
    mpu6050_set_accel_offsets(ACCEL_OFFSET_X, ACCEL_OFFSET_Y, ACCEL_OFFSET_Z);
    Serial.printf("Accel offsets set: X=%.4f Y=%.4f Z=%.4f\n",
                  ACCEL_OFFSET_X, ACCEL_OFFSET_Y, ACCEL_OFFSET_Z);

    // Re-zero gyro every boot (must be stationary, doesn't need to be level)
    Serial.println("Calibrating gyro (keep drone STATIONARY)...");
    digitalWrite(LED_BUILTIN, HIGH);
    mpu6050_calibrate_gyro();
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Gyro calibration complete.");
#endif
    
    // ===== HMC5883L =====
    Serial.println("Init HMC5883L...");
    hmc5883l_init();

#if CALIBRATE_MAG
    // Calibration mode: rotate the drone in a figure-8 to capture hard/soft iron offsets.
    Serial.println("\n=== MAGNETOMETER CALIBRATION ===");
    Serial.println("Rotate drone in a FIGURE-8 covering all angles.");
    Serial.println("Starting in 3 seconds...");
    delay(3000);

    digitalWrite(LED_BUILTIN, HIGH);
    hmc5883l_calibrate();
    digitalWrite(LED_BUILTIN, LOW);

    float ox, oy, oz, sx, sy, sz;
    hmc5883l_get_calibration(ox, oy, oz, sx, sy, sz);

    Serial.println("\n=== COMPLETE - copy to main.cpp ===");
    Serial.printf("#define MAG_OFFSET_X  %.2ff\n", ox);
    Serial.printf("#define MAG_OFFSET_Y  %.2ff\n", oy);
    Serial.printf("#define MAG_OFFSET_Z  %.2ff\n", oz);
    Serial.printf("#define MAG_SCALE_X   %.4ff\n", sx);
    Serial.printf("#define MAG_SCALE_Y   %.4ff\n", sy);
    Serial.printf("#define MAG_SCALE_Z   %.4ff\n", sz);
    Serial.println("Then set CALIBRATE_MAG=false and re-upload.");

    while (true) { delay(100); }
#else
    hmc5883l_set_offsets(MAG_OFFSET_X, MAG_OFFSET_Y, MAG_OFFSET_Z);
    hmc5883l_set_scales(MAG_SCALE_X, MAG_SCALE_Y, MAG_SCALE_Z);

    Serial.printf("Mag Offsets Loaded: X=%.2f Y=%.2f Z=%.2f\n", MAG_OFFSET_X, MAG_OFFSET_Y, MAG_OFFSET_Z);
#endif

    // ===== CONTROL =====
    initFilters();
    initPIDs();
    initRateCurves();
    initTPA();
    fusion.begin(GYRO_LOOP_HZ / FUSION_DIVIDER);

    // ===== STATE =====
    portENTER_CRITICAL(&stateMux);
    rcThrottle = ESC_MIN_US; rcRoll = rcPitch = rcYaw = 0;
    flightMode = MODE_STABILIZE; motorsArmed = false; rcConnected = false;
    levelTrimRoll = LEVEL_TRIM_ROLL_DEG;
    levelTrimPitch = LEVEL_TRIM_PITCH_DEG;
    portEXIT_CRITICAL(&stateMux);
    
    Serial.printf("Level trim initialized: Roll=%.2f° Pitch=%.2f°\n", 
                  LEVEL_TRIM_ROLL_DEG, LEVEL_TRIM_PITCH_DEG);
    
    // ===== IBUS =====
    Serial.println("Init IBUS...");
    ibus.failsafeValues[CH_THROTTLE-1] = IBUS_MIN;
    ibus.failsafeValues[CH_ROLL-1] = IBUS_MID;
    ibus.failsafeValues[CH_PITCH-1] = IBUS_MID;
    ibus.failsafeValues[CH_YAW-1] = IBUS_MID;
    ibus.failsafeValues[CH_ARM_SWITCH-1] = IBUS_MIN;
    ibus.begin();
    
    // ===== WIFI =====
    Serial.println("Init WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) { delay(500); Serial.print("."); }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) Serial.println("http://" + WiFi.localIP().toString());

    setupWebServer();

    Serial.println("\n=== Commands ===");
    Serial.println("ARM DISARM ACRO STAB STATUS PIDS");
    Serial.println("L<s>  - Log for s seconds");
    Serial.println("S     - Stop logging");
    Serial.println("D     - Dump log data");
    Serial.println("GF<hz> / GFON / GFOFF - Gyro filter");
    Serial.println("TRIMR<deg> - Set roll trim");
    Serial.println("TRIMP<deg> - Set pitch trim");
    Serial.println("TRIM      - Show current trim");
    Serial.println("Manual: T1200 R0.5 P-0.3 Y0.1");
    Serial.println("================================\n");

    // Flight on core 1 (isolated); comms+IBUS on core 0
    xTaskCreatePinnedToCore(flightTask, "Flight", 8192, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(commsTask, "Comms", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(ibusTask, "IBUS", 4096, NULL, 5, NULL, 0);

    vTaskDelete(NULL);
}


void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }