#ifndef BATTERY_H
#define BATTERY_H

class Battery
{
  public:
    Battery(int pin_number);

    double voltage() const;
    bool isLow() const;

    double adjustVoltage(double unadjusted_voltage) const;

  private:
    static constexpr double kVoltsPerAnalogReading = 0.008395;
    static constexpr double kLowVoltage = 7.7;

    int pin_;
}

extern gBattery;

#endif
