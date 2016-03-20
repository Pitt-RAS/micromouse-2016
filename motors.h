#ifndef MOTORS_H
#define MOTORS_H

class Motor {
  private:
    int pin_f_, pin_pwm_f_, pin_b_, pin_pwm_b_;
    bool forward_state_f_, forward_state_b_;
  public:
    Motor(int motor_f_pin, int motor_f_pwm_pin, bool motor_f_forward_state,
          int motor_b_pin, int motor_b_pwm_pin, bool motor_b_forward_state);
    void Set(float accel, float current_velocity);
};

extern Motor motor_l, motor_r;

#endif
