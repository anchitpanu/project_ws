#ifndef DIGGING_H
#define DIGGING_H

#define DRILL_STEP_PIN_DIR  14
#define DRILL_STEP_PIN_PLS  12

// // Stepper speed (RPM). 8–12 is safe for 28BYJ-48
// #define STEPPER_RPM   10

#define MOTOR_STEPS_PER_REV 200     // NEMA17 1.8° = 200
#define MICROSTEP            1      // DIP on A4988/DRV8825/TMC

#define MODULE = 0.5f           // e.g., 0.5 module gear
#define PINION_TEETH = 8        // e.g., 8 teeth pinion gear
#define GEAR_RATIO = 64         // e.g., 1/64 gear ratio
#define TRAVEL_MM_PER_REV = π * MODULE * PINION_TEETH * GEAR_RATIO

#define TRAVEL_CM 10.0f          // e.g., 20 cm total travel (adjust as needed)
#define STEPS_PER_CM = (MOTOR_STEPS_PER_REV * MICROSTEP) / (TRAVEL_MM_PER_REV / 10)
// #define STEPS_PER_CM 127.3f      // set correctly for your mechanism!

#define JOY_PRESS_THRESH     0.5f     // press threshold for z=1.0

// // Optional soft limits (top = 0 mm, positive = down)
// static const float MAX_TRAVEL_MM = 400.0f;          // e.g., 40 cm stroke. Set to your real value.


const int SERVO_CLOSED = 0;     // fully closed position
const int SERVO_OPENED = 60;    // opened position (adjust as needed)

#endif