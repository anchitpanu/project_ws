#ifndef SPIN_H
#define SPIN_H

const float TWIST_THRESH   = 0.2f;   // adjust as needed; velocity threshold to start spinning
const float TWIST_DEADZONE = 0.05f;

// ULN2003 IN1..IN4 -> ESP32 pins (EDIT to match your wiring)
#define STEPPER_IN1  8
#define STEPPER_IN2  10
#define STEPPER_IN3  9
#define STEPPER_IN4  11

// 28BYJ-48: ~2048 steps per 360° (depends on library/gearbox)
#define STEPS_PER_REV 2048
#define DEG_PER_STEP  (360.0 / STEPS_PER_REV)
#define STEPS_PER_36  ((int)round(36.0 / DEG_PER_STEP))

// Stepper speed (RPM). 8–12 is safe for 28BYJ-48
#define STEPPER_RPM   10

// How many steps to perform per control tick (non-blocking chunk)
#define STEP_CHUNK_PER_TICK  10

#endif
