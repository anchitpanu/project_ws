#ifndef DIGGING_H
#define DIGGING_H

#define DRILL_STEP_PIN_DIR  25
#define DRILL_STEP_PIN_PLS  26

// Stepper speed (RPM). 8–12 is safe for 28BYJ-48
#define STEPPER_RPM   10

static const float STEPS_PER_MM = 40.0f;            // set correctly for your mechanism!

static const float PRESS_TRAVEL_MM = 200.0f;        // 20 cm

// Optional soft limits (top = 0 mm, positive = down)
static const float MAX_TRAVEL_MM = 400.0f;          // e.g., 40 cm stroke. Set to your real value.

// Step timing
static const uint32_t STEP_PULSE_HIGH_US = STEP_PULSE_HIGH_US;   // you already have this const
static const uint32_t STEP_PERIOD_US     = STEP_PERIOD_US;       // you already have this const

// Trigger threshold (treat this as a "button" from joystick)
static const float PRESS_THRESH = 0.8f;   // consider pressed when |z| >= 0.8 (adjust if needed)

static bool    prev_pressed = false;      // edge detect
static bool    move_down_next = true;     // toggles every valid press
static long    current_steps = 0;         // 0 at top; +down (software odometer)

const int SERVO_CLOSED = 0;     // fully closed position
const int SERVO_OPENED = 60;    // opened position (adjust as needed)

#endif