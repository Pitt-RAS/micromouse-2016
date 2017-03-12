#include <cmath>
#include <Arduino.h>
#include "Battery.h"
#include "Motor.h"

Motor gMotorLF(16, 20, false);
Motor gMotorLB(17, 21, true);
Motor gMotorRF(18, 22, true);
Motor gMotorRB(19, 23, false);

Motor::Motor(int direction_pin, int pwm_pin, bool reversed) :
  direction_pin_(direction_pin), pwm_pin_(pwm_pin), reversed_(reversed)
{}

void Motor::voltage(double value)
{
  value = gBattery.adjustVoltage(value);

  if (value < -1.0) value = -1.0;
  if (value >  1.0) value =  1.0;

  bool direction = value >= 0.0;

  if (reversed_) direction = !direction;

  double magnitude = std::fabs(kSpeedSteps * value);

  digitalWrite(direction_pin_, direction);
  analogWrite(pwm_pin_, magnitude);
}
