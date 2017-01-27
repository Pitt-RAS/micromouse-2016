#ifndef MOTORS_H
#define MOTORS_H

class Motor {
  private:
    int pin_, pin_pwm_;
    bool forward_state_;
  public:
    Motor(int motor_f_pin, int motor_f_pwm_pin, bool motor_f_forward_state);
    void Set(float accel, float current_velocity);
};

extern Motor motor_lf, motor_lb, motor_rf, motor_rb;

#endif
