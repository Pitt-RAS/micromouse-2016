#include <Arduino.h>
#include "conf.h"
#include "motors.h"



struct motor motor_a, motor_b;

static void motor_init(struct motor *motor, int pin1, int pin2, int pinpwm);

void motors_init() {
    motor_init(&motor_a, MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_AP_PIN);
    motor_init(&motor_b, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_BP_PIN);
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

void motor_set(struct motor *motor, float speed)
{
    int pin1_state, pin2_state;
    int speed_raw;

    if (speed < -1.0) 
        speed = -1.0;
    else if (speed > 1.0)
        speed = 1.0;

    speed_raw = (int)(round(PWM_SPEED_STEPS * speed));

    if (speed_raw < 0) 
        speed_raw *= -1;

    if (speed > 0.0) {
        pin1_state = HIGH;
        pin2_state = LOW;
    }
    else if (speed < 0.0) {
        pin1_state = LOW;
        pin2_state = HIGH;
    }
    else {
        pin1_state = LOW;
        pin2_state = LOW;
    }

    digitalWrite(motor->pin1, pin1_state);
    digitalWrite(motor->pin2, pin2_state);
    analogWrite(motor->pinpwm, speed_raw);
}
