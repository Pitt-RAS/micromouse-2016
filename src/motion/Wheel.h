#ifndef WHEEL_H
#define WHEEL_H

#include "../device/Motor.h"
#include "../device/Encoder.h"
#include "Profile.h"

namespace Motion {

class Wheel
{
  public:
    Wheel(Motor &motor, Encoder &encoder);

    void reference(LinearPoint point);
    LinearPoint reference() const;

    void update(TimeUnit time);

    void reset();

  private:
    Motor &motor_;
    Encoder &encoder_;

    LinearPoint reference_ = LinearPoint::zero();
};

}

#endif
