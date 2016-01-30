#ifndef MOTORS_H
#define MOTORS_H

class Motor {
  private:
    int pin1_, pin2_, pin_pwm_;
  public:
    Motor(int pin1, int pin2, int pwm_pin);
    void Set(float accel, float current_velocity);
};

extern Motor motor_l, motor_r;

#endif
