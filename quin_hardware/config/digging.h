#ifndef DIGGING_H
#define DIGGING_H

const float TWIST_THRESH   = 0.2f;   // adjust as needed; velocity threshold to start spinning
const float TWIST_DEADZONE = 0.05f;

#define DRILL_STEP_PIN_DIR  25
#define DRILL_STEP_PIN_PLS  26

#define MOTOR_STEPS_PER_REV 200     // NEMA17 1.8° = 200
#define MICROSTEP            16     // DIP on A4988/DRV8825/TMC
#define STEPS_PER_REV         (MOTOR_STEPS_PER_REV * MICROSTEPPING)

#define GEAR_MODULE           1.0f
#define PINION_TEETH          20
#define LINEAR_PER_REV_MM     (M_PI * GEAR_MODULE * PINION_TEETH)

#define STEPS_PER_MM          (STEPS_PER_REV / LINEAR_PER_REV_MM)

#define DIST_PER_TRIGGER_MM   5.0f  // how many mm per trigger press

#define STEP_PULSE_HIGH_US    5     // width of STEP pulse (3–5us typical)  
#define STEP_PERIOD_US   2000       // adjust speed here (lower = faster)

// // Stepper speed (RPM). 8–12 is safe for 28BYJ-48
// #define STEPPER_RPM   10

const int SERVO_CLOSED = 0;     // fully closed position
const int SERVO_OPENED = 60;    // opened position (adjust as needed)

#endif