#ifndef DIGGING_H
#define DIGGING_H

#define DRILL_STEP_PIN_DIR  25
#define DRILL_STEP_PIN_PLS  26

// // Stepper speed (RPM). 8–12 is safe for 28BYJ-48
// #define STEPPER_RPM   10

#define TRAVEL_CM 20.0f          // e.g., 20 cm total travel (adjust as needed)
#define STEPS_PER_CM 200.0f      // set correctly for your mechanism!

// // Optional soft limits (top = 0 mm, positive = down)
// static const float MAX_TRAVEL_MM = 400.0f;          // e.g., 40 cm stroke. Set to your real value.

// Step timing
static const uint32_t STEP_PULSE_HIGH_US = 500  
static const uint32_t STEP_PERIOD_US     = 1000       

// Trigger threshold (treat this as a "button" from joystick)
static const float JOY_PRESS_THRESH = 0.8f;   // consider pressed when |z| >= 0.8 (adjust if needed)


const int SERVO_CLOSED = 0;     // fully closed position
const int SERVO_OPENED = 60;    // opened position (adjust as needed)

#endif