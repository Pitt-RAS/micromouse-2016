#ifndef MOTORS_H
#define MOTORS_H

// describes a motor
struct motor {
    int pin1, pin2, pinpwm;
};

extern struct motor motor_a, motor_b;

// Initialize motors
void motors_init();

// Run a motor at speed, where -1.0 <= speed <= 1.0.
void motor_set(struct motor *motor, float speed);

#endif
