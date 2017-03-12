#ifndef MOTOR_H
#define MOTOR_H

class Motor
{
  public:
    Motor(int direction_pin, int pwm_pin, bool reversed);

    // from minimum -1.0 to maximum 1.0
    void voltage(double value);

  private:
    static const int kSpeedSteps = 1023;

    const int direction_pin_;
    const int pwm_pin_;
    const bool reversed_;
};

extern Motor gMotorLF, gMotorLB, gMotorRF, gMotorRB;

#endif
