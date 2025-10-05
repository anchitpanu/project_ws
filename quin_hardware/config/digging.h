#ifndef DIGGING_H
#define DIGGING_H

#define DRILL_STEP_PIN_DIR  14
#define DRILL_STEP_PIN_PLS  12

// // Stepper speed (RPM). 8–12 is safe for 28BYJ-48
// #define STEPPER_RPM   10

#define TRAVEL_CM 5.0f          // e.g., 20 cm total travel (adjust as needed)
#define STEPS_PER_CM 127.3f      // set correctly for your mechanism!

#define JOY_PRESS_THRESH     0.5f     // press threshold for z=1.0

// // Optional soft limits (top = 0 mm, positive = down)
// static const float MAX_TRAVEL_MM = 400.0f;          // e.g., 40 cm stroke. Set to your real value.


const int SERVO_CLOSED = 0;     // fully closed position
const int SERVO_OPENED = 60;    // opened position (adjust as needed)

#endif