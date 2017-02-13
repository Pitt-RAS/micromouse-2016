#ifndef WHEEL_H
#define WHEEL_H

#include "../device/Motor.h"
#include "../device/Encoder.h"
#include "PIDFunction.h"
#include "Profile.h"

namespace Motion {

struct WheelOptions
{
  PIDParameters pid_parameters;
};

class Wheel
{
  public:
    Wheel(WheelOptions options, Motor &motor, Encoder &encoder);

    void reference(LinearPoint point);
    LinearPoint reference() const;

    void update(TimeUnit time);

    void reset();

  private:
    Motor &motor_;
    Encoder &encoder_;

    LinearPoint reference_ = {
      LengthUnit::zero(), LengthUnit::zero(), LengthUnit::zero()
    };
};

}

#endif
