#include "motors.h"

#define SPEED_STEPS 255

void motors_init() {
    motor_init(&motor_a, MOTOR_A1, MOTOR_A2, MOTOR_AP);
    motor_init(&motor_b, MOTOR_B1, MOTOR_B2, MOTOR_BP);
}

static void motor_init(struct motor *motor, int pin1, int pin2, int pinpwm)
{
    motor->pin1 = pin1;
    motor->pin2 = pin2;
    motor->pinpwm = pinpwm;

    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pinpwm, OUTPUT);
}

void motor_run(struct motor *motor, float speed)
{
    int pin1_state, pin2_state;
    int speed_raw;

    if (speed < -1.0 || speed > 1.0)
        speed = 0.0;

    speed_raw = (int)(SPEED_STEPS * speed);

    if (speed_raw < 0)
        speed_raw *= -1;

    pin1_state = LOW;
    pin2_state = LOW;

    if (speed > 0.0)
        pin1_state = HIGH;
    else if (speed < 0.0)
        pin2_state = HIGH;

    digitalWrite(motor->pin1, pin1_state);
    digitalWrite(motor->pin2, pin2_state);
    analogWrite(motor->pinpwm, speed_raw);
}

void motor_brake(struct motor *motor, float strength)
{
    int strength_raw;

    if (strength < 0.0 || strength > 1.0)
        strength = 0.0;

    strength_raw = (int)(SPEED_STEPS * strength);

    digitalWrite(motor->pin1, HIGH);
    digitalWrite(motor->pin2, HIGH);
    analogWrite(motor->pinpwm, strength_raw);
}

void motor_coast(struct motor *motor)
{
    digitalWrite(motor->pin1, LOW);
    digitalWrite(motor->pin2, LOW);
    analogWrite(motor->pinpwm, 0);
}
