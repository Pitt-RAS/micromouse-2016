#include <Arduino.h>
#include "Battery.h"

Battery gBattery(A14);

Battery::Battery(int pin_number) : pin_(pin_number)
{}

double Battery::voltage() const
{
  return kVoltsPerAnalogReading * analogRead(pin_);
}

bool Battery::isLow() const
{
  return voltage() <= kLowVoltage;
}

double Battery::adjustVoltage(double unadjusted_voltage) const
{
  return unadjusted_voltage * kLowVoltage / voltage();
}
