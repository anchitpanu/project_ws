#ifndef SPIN_H
#define SPIN_H

const float TWIST_THRESH   = 0.2f;   // adjust as needed; velocity threshold to start spinning
const float TWIST_DEADZONE = 0.05f;

// ULN2003 IN1..IN4 -> ESP32 pins (EDIT to match your wiring)
#define DIR_PIN  17
#define PLS_PIN  16

#define MOTOR_STEPS_PER_REV 200     // NEMA17 1.8° = 200
#define MICROSTEP            1      // DIP on A4988/DRV8825/TMC

// Total steps per revolution
#define STEPS_PER_REV  (MOTOR_STEPS_PER_REV * MICROSTEP)        // 200*16 = 3200
#define STEPS_PER_36   ((int)round(STEPS_PER_REV / 10.0f))      // 3200/10 = 320

#define STEP_PULSE_HIGH_US  4      // กว้างพัลส์ STEP (3–5us ทั่วไป)
#define STEP_PERIOD_US      5000   // adjust speed here (lower = faster)

// 28BYJ-48: ~2048 steps per 360° (depends on library/gearbox)
// #define STEPS_PER_REV 200
// #define DEG_PER_STEP  (360.0 / STEPS_PER_REV)
// #define STEPS_PER_36  ((int)round(36.0 / DEG_PER_STEP))

// Stepper speed (RPM). 8–12 is safe for 28BYJ-48
#define STEPPER_RPM   10

// How many steps to perform per control tick (non-blocking chunk)
#define MAX_STEPS_PER_TICK  10

#endif
