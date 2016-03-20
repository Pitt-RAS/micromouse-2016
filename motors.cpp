#include <Arduino.h>
#include "conf.h"
#include "motors.h"

// input desired force and current speed
static float idealMotorOutput(float force, float velocity) {
  float required_current, back_emf;
  required_current = force / FORCE_PER_AMP;
  back_emf = velocity / VELOCITY_PER_VBEMF;
  return ((required_current * RATED_INTERNAL_RESISTANCE + back_emf) / BATTERY_VOLTAGE);
}

Motor motor_l (MOTOR_LF_DIRECTION_PIN, MOTOR_LF_PWM_PIN);
Motor motor_r (MOTOR_RF_DIRECTION_PIN, MOTOR_RF_PWM_PIN);

Motor::Motor(int pin, int pwm_pin) {
  pin_ = pin;
  pin_pwm_ = pwm_pin;

  pinMode(pin_, OUTPUT);
  pinMode(pin_pwm_, OUTPUT);
}

void Motor::Set(float accel, float current_velocity) {
  float force;
  float speed;
  int pin_state;
  int speed_raw;

  if (current_velocity > 0) {
    force = (ROBOT_MASS * accel + FRICTION_FORCE) / NUMBER_OF_MOTORS;
  } else if (current_velocity < 0) {
    force = (ROBOT_MASS * accel - FRICTION_FORCE) / NUMBER_OF_MOTORS;
  } else {
    force = (ROBOT_MASS * accel) / NUMBER_OF_MOTORS;
  }

  speed = idealMotorOutput(force, current_velocity);
  speed = constrain(speed, -1, 1);

  speed_raw = abs((int)(round(PWM_SPEED_STEPS * speed)));

  if (speed > 0.0) {
    pin_state = HIGH;
  } else {
    pin_state = LOW;
  }

  digitalWrite(pin_, pin_state);
  analogWrite(pin_pwm_, speed_raw);
}
