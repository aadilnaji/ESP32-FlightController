#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/** Compile-time configuration: pins, timing, control limits, gains, filter cutoffs. */

// ===== WIFI =====
#define WIFI_SSID     "ssid"
#define WIFI_PASSWORD "password"

// ===== PINS =====
#define ESC1_PIN 25
#define ESC2_PIN 26
#define ESC3_PIN 27
#define ESC4_PIN 14
#define IBUS_RX_PIN 16
#define IBUS_TX_PIN -1
#define I2C_SDA 21
#define I2C_SCL 22
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// ===== ESC =====
#define ESC_MIN_US 1000
#define ESC_MAX_US 1940
#define ESC_PWM_FREQ_HZ 490
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
#define GYRO_LOOP_HZ 1000
#define GYRO_LOOP_US (1000000 / GYRO_LOOP_HZ)
#define FUSION_DIVIDER 4
#define MAG_DIVIDER 67

// ===== CONTROL LIMITS =====
#define MAX_ANGLE_DEG 30.0f
#define MAX_RATE_DPS 500.0f

// ===== MADGWICK =====
#define MADGWICK_BETA_STARTUP 0.5f
#define MADGWICK_BETA_FLIGHT 0.1f

// ===== RATE CURVE =====
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

// ===== LOGGER =====
#define LOG_SAMPLES 1500
#define LOG_RAW_GYRO
#define LOG_FILTERED_GYRO
#define LOG_FUSED_ANGLES
#define LOG_PID_OUTPUTS
#define LOG_PID_COMMANDS
#define LOG_MOTOR_COMMANDS

// ===== SHARED TYPES =====
enum FlightMode { MODE_ACRO = 0, MODE_STABILIZE = 1 };

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295f
#endif

#endif
