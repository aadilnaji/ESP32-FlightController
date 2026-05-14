/** ESP32 flight controller: cascaded 250Hz angle / 1000Hz rate loops, Madgwick fusion, IBUS RC, WiFi telemetry. 
 *  
 *  Author: Aadil Naji
 *  Credits: 
 *  - Sebastian Madgwick (AHRS algorithm)
 *  - Betaflight (PID and filter design inspiration).
*/

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "Config.h"
#include "Calibration.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "Madgwick.h"
#include "Filters.h"
#include "PID.h"
#include "IBUSReceiver.h"
#include "Logger.h"
#include "Telemetry.h"
#include "dashboard.h"

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

volatile uint32_t loopTimeMin = UINT32_MAX;
volatile uint32_t loopTimeMax = 0;
volatile uint64_t loopTimeSum = 0;
volatile uint32_t loopCount = 0;
volatile uint32_t loopOverruns = 0;
volatile uint32_t lastLoopTime = 0;

volatile int rcArmSwitchVal = 1000;
volatile int rcModeSwitchVal = 1000;

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
        
        if (Logger::isActive()) {
            FlightFrame f;
            f.timestamp_us = loopStart;
            f.gx = GyroX;   f.gy = GyroY;   f.gz = GyroZ;
            f.gxf = GyroXf; f.gyf = GyroYf; f.gzf = GyroZf;
            f.roll = fusedRoll; f.pitch = fusedPitch; f.yaw = fusedYaw;
            f.pidR = pidRoll;   f.pidP = pidPitch;   f.pidY = pidYaw;
            f.cmdR = rollCmd;   f.cmdP = pitchCmd;   f.cmdY = yawCmd;
            f.throttle = localThrottle;
            f.m1 = out1; f.m2 = out2; f.m3 = out3; f.m4 = out4;
            Logger::sample(f);
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

            TelemetryData t = {};
            t.roll = fusedRoll;
            t.pitch = fusedPitch;
            t.yaw = fusedYaw;
            t.angleErrRoll = angleErrRoll;
            t.angleErrPitch = angleErrPitch;

            t.gyroX = GyroX;   t.gyroY = GyroY;   t.gyroZ = GyroZ;
            t.gyroXf = GyroXf; t.gyroYf = GyroYf; t.gyroZf = GyroZf;

            t.accelX = AccX * 9.81f;
            t.accelY = AccY * 9.81f;
            t.accelZ = AccZ * 9.81f;
            t.magX = MagX; t.magY = MagY; t.magZ = MagZ;
            t.magValid = magOk;

            t.rateSetRoll = rateSetRoll;
            t.rateSetPitch = rateSetPitch;
            t.rateSetYaw = rateSetYaw;
            t.rateErrRoll = rateErrRoll;
            t.rateErrPitch = rateErrPitch;
            t.rateErrYaw = rateErrYaw;

            t.pTermRoll = pR; t.pTermPitch = pP; t.pTermYaw = pY;
            t.iTermRoll = iR; t.iTermPitch = iP; t.iTermYaw = iY;
            t.dTermRoll = dR; t.dTermPitch = dP; t.dTermYaw = dY;
            t.fTermRoll = fR; t.fTermPitch = fP; t.fTermYaw = fY;
            t.pidTotalRoll = rollCmd;
            t.pidTotalPitch = pitchCmd;
            t.pidTotalYaw = yawCmd;

            t.mixRoll = rollCmd;
            t.mixPitch = pitchCmd;
            t.mixYaw = yawCmd;
            t.throttle = localThrottle;
            t.m1 = out1; t.m2 = out2; t.m3 = out3; t.m4 = out4;
            t.motorSaturated = saturated;

            t.rcRoll = localRcRoll;
            t.rcPitch = localRcPitch;
            t.rcYaw = localRcYaw;
            t.rcThrottle = localThrottle;

            t.armed = localArmed;
            t.mode = localMode;
            t.fusionConverged = fusionConverged;
            t.airmodeActive = airmodeAllowed;
            t.tpaFactor = tpaFactor;

            t.trimRoll = trimR;
            t.trimPitch = trimP;

            t.headlessActive = headlessModeActive;
            t.headlessRefYaw = headlessReferenceYaw;

            t.loopTime = loopTime;
            t.loopMin = loopTimeMin;
            t.loopMax = loopTimeMax;
            t.loopAvg = loopCount > 0 ? (uint32_t)(loopTimeSum / loopCount) : 0;
            t.loopJitter = jitter;
            t.overruns = loopOverruns;

            Telemetry::publish(t);
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
        server.send(200, "application/json", Telemetry::buildJson());
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
    
    if (s.startsWith("L")) { Logger::start(s.substring(1).toInt() > 0 ? s.substring(1).toInt() : 5); return; }
    if (s == "S") { Logger::stop(); return; }
    if (s == "D") { if (Logger::hasData()) Logger::dump(); return; }
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
        TelemetryData snap;
        Telemetry::snapshot(snap);
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