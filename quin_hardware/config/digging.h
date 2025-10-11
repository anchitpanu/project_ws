#ifndef DIGGING_H
#define DIGGING_H

// ---------------- Pins ----------------
#define DRILL_STEP_PIN_DIR  27
#define DRILL_STEP_PIN_PLS  26

// ---------------- Mechanics ----------------
// Motor & driver
#define MOTOR_STEPS_PER_REV   200    // NEMA17 1.8° -> 200 steps/rev
#define MICROSTEP              1      // A4988/DRV8825/TMC microstepping (1,2,4,8,16, ...)

// Rack & pinion (module gear)
#define MODULE                0.5f    // module m (mm per tooth pitch / π)
#define PINION_TEETH          25      // pinion tooth count
#define GEAR_RATIO            1.0f    // motor_rev : pinion_rev (e.g., 64 means 64 motor rev per 1 pinion rev)

// π (avoid M_PI portability issues on Arduino)
#define PI_F                  3.14159265f

// Travel per ONE pinion revolution (in mm)
#define TRAVEL_MM_PER_PINION_REV   (PI_F * (MODULE) * (PINION_TEETH))

// Steps per ONE pinion revolution (include gearbox & microstepping)
#define STEPS_PER_PINION_REV       ((MOTOR_STEPS_PER_REV) * (MICROSTEP) * (GEAR_RATIO))

// Steps per mm (and per cm)
#define STEPS_PER_MM               ((STEPS_PER_PINION_REV) / (TRAVEL_MM_PER_PINION_REV))
#define STEPS_PER_CM               (STEPS_PER_MM * 10.0f)

// ---------------- Motion Commanding ----------------
#define TRAVEL_CM              10.0f   // commanded stroke per press (e.g., 10 cm)

// Joystick threshold
#define JOY_PRESS_THRESH       0.5f

// ---------------- Servo positions ----------------

#define SERVO_PIN  13
static const int SERVO_CLOSED = 0;     // fully closed position
static const int SERVO_OPENED = 50;    // opened position (tune as needed)

#endif
