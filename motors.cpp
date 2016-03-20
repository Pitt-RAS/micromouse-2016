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

Motor motor_l (MOTOR_LF_DIRECTION_PIN, MOTOR_LF_PWM_PIN, MOTOR_LF_FORWARD_STATE,
               MOTOR_LB_DIRECTION_PIN, MOTOR_LB_PWM_PIN, MOTOR_LB_FORWARD_STATE);
Motor motor_r (MOTOR_RF_DIRECTION_PIN, MOTOR_RF_PWM_PIN, MOTOR_RF_FORWARD_STATE,
               MOTOR_RB_DIRECTION_PIN, MOTOR_RB_PWM_PIN, MOTOR_RB_FORWARD_STATE);

Motor::Motor(int motor_f_pin, int motor_f_pwm_pin, bool motor_f_forward_state,
             int motor_b_pin, int motor_b_pwm_pin, bool motor_b_forward_state) {
  pin_f_ = motor_f_pin;
  pin_pwm_f_ = motor_f_pwm_pin;
  forward_state_f_ = motor_f_forward_state;
  pin_b_ = motor_b_pin;
  pin_pwm_b_ = motor_b_pwm_pin;
  forward_state_b_ = motor_b_forward_state;

  pinMode(pin_f_, OUTPUT);
  pinMode(pin_pwm_f_, OUTPUT);
  pinMode(pin_b_, OUTPUT);
  pinMode(pin_pwm_b_, OUTPUT);
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

  digitalWrite(pin_f_, pin_state ^ forward_state_f_ ^ 1);
  analogWrite(pin_pwm_f_, speed_raw);
  digitalWrite(pin_b_, pin_state ^ forward_state_b_ ^ 1);
  analogWrite(pin_pwm_b_, speed_raw);
}
