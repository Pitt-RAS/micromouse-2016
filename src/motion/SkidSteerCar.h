#ifndef SKID_STEER_CAR_H
#define SKID_STEER_CAR_H

#include "Wheel.h"
#include "BodyWithPoint.h"
#include "Profile.h"

namespace Motion {

struct WheelOnBody
{
  Wheel wheel;
  BodyWithPoint body;
};

template <size_t length>
class SkidSteerCar
{
  public:
    // not sure whether to store pointer to array, or to copy array...
    SkidSteerCar(WheelOnBody wheels_on_body_[length]);

    void reference(LinearRotationalPoint point);
    LinearRotationalPoint reference() const;

    void update(TimeUnit dt);

  private:
    WheelOnBody wheels_on_body_[length];
};

}

#endif
