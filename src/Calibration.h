#ifndef CALIBRATION_H
#define CALIBRATION_H

/** Per-airframe calibration values. Flip CALIBRATE_* to true to recapture; serial prints new defines on completion. */

// ===== MAGNETOMETER =====
#define CALIBRATE_MAG   false
#define MAG_OFFSET_X  44.60f
#define MAG_OFFSET_Y  -192.40f
#define MAG_OFFSET_Z  -62.40f
#define MAG_SCALE_X   0.9775f
#define MAG_SCALE_Y   0.9881f
#define MAG_SCALE_Z   1.0364f

// ===== ACCELEROMETER =====
#define CALIBRATE_ACCEL false
#define ACCEL_OFFSET_X 0.036650f
#define ACCEL_OFFSET_Y 0.049102f
#define ACCEL_OFFSET_Z -0.102405f

// ===== LEVEL TRIM =====
// Positive roll = drone needs right roll to hover level; positive pitch = forward.
#define LEVEL_TRIM_ROLL_DEG   -1.2f
#define LEVEL_TRIM_PITCH_DEG  -1.2f

#endif
