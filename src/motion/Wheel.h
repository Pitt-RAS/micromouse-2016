#ifndef WHEEL_H
#define WHEEL_H

#include "../device/Motor.h"
#include "../device/Encoder.h"
#include "Profile.h"

namespace Motion {

class Wheel
{
  public:
    Wheel(Motor &motor, Encoder &Encoder);

    void reference(LinearPoint point);
    LinearPoint reference() const;

    void update(TimeUnit dt);

    void reset();
};

}

#endif
