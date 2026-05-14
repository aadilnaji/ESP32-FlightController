#include "Logger.h"

// Stored record. Field set depends on which LOG_* macros are defined in Config.h.
struct LoopLog {
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
};

static LoopLog logs[LOG_SAMPLES];
static volatile bool active = false;
static volatile bool complete = false;
static volatile uint32_t logIndex = 0;
static volatile uint32_t startMs = 0;
static volatile uint32_t durationMs = 0;

static inline int16_t scaleToInt16(float val, float scale) {
    return (int16_t)constrain((int32_t)roundf(val * scale), -32768, 32767);
}

static void printHeader() {
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

namespace Logger {

void start(int seconds) {
    logIndex = 0;
    active = true;
    complete = false;
    startMs = millis();
    durationMs = (uint32_t)seconds * 1000u;
    Serial.printf("Logging for %ds\n", seconds);
}

void stop() {
    active = false;
    complete = true;
    Serial.printf("Logged %d samples\n", logIndex);
}

bool isActive() { return active; }
bool hasData()  { return complete; }

void dump() {
    Serial.println("\n===== DATA DUMP =====");
    printHeader();
    for (uint32_t i = 0; i < logIndex; i++) {
        LoopLog& e = logs[i];
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

void sample(const FlightFrame& f) {
    if (!active || logIndex >= LOG_SAMPLES) return;

    LoopLog& e = logs[logIndex++];
    e.timestamp_us = (uint32_t)f.timestamp_us;
#ifdef LOG_RAW_GYRO
    e.gyro_x = scaleToInt16(f.gx, 100);
    e.gyro_y = scaleToInt16(f.gy, 100);
    e.gyro_z = scaleToInt16(f.gz, 100);
#endif
#ifdef LOG_FILTERED_GYRO
    e.gyro_xf = scaleToInt16(f.gxf, 100);
    e.gyro_yf = scaleToInt16(f.gyf, 100);
    e.gyro_zf = scaleToInt16(f.gzf, 100);
#endif
#ifdef LOG_FUSED_ANGLES
    e.roll  = scaleToInt16(f.roll, 100);
    e.pitch = scaleToInt16(f.pitch, 100);
    e.yaw   = scaleToInt16(f.yaw, 100);
#endif
#ifdef LOG_PID_OUTPUTS
    e.pid_roll  = scaleToInt16(f.pidR, 10);
    e.pid_pitch = scaleToInt16(f.pidP, 10);
    e.pid_yaw   = scaleToInt16(f.pidY, 10);
#endif
#ifdef LOG_PID_COMMANDS
    e.cmd_roll  = scaleToInt16(f.cmdR, 10);
    e.cmd_pitch = scaleToInt16(f.cmdP, 10);
    e.cmd_yaw   = scaleToInt16(f.cmdY, 10);
#endif
#ifdef LOG_MOTOR_COMMANDS
    e.throttle = f.throttle;
    e.m1 = f.m1; e.m2 = f.m2; e.m3 = f.m3; e.m4 = f.m4;
#endif

    if (millis() - startMs >= durationMs) {
        active = false;
        complete = true;
    }
}

}  // namespace Logger
