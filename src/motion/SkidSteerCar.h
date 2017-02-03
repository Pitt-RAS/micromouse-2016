#ifndef SKID_STEER_CAR_H
#define SKID_STEER_CAR_H

#include "Wheel.h"
#include "PointOnBody.h"
#include "Profile.h"

namespace Motion {

struct WheelOnBody
{
  Wheel wheel;
  PointOnBody point_on_body;
};

template <size_t length>
class SkidSteerCar
{
  public:
    SkidSteerCar(WheelOnBody *wheels_on_body);

    void reference(LinearRotationalPoint point);
    LinearRotationalPoint reference() const;

    void update(TimeUnit time);

    void reset();

  private:
    WheelOnBody *wheels_on_body_;

    LinearRotationalPoint reference_ = LinearRotationalPoint::zero();
};

template <size_t length>
SkidSteerCar<length>::SkidSteerCar(WheelOnBody wheels_on_body[]) :
  wheels_on_body_(wheels_on_body)
{}

template <size_t length>
void SkidSteerCar<length>::reference(LinearRotationalPoint point)
{
  for (size_t i = 0; i < length; i++) {
    Wheel &wheel = wheels_on_body_[i].wheel;
    PointOnBody &point_on_body = wheels_on_body_[i].point_on_body;

    wheel.reference(point_on_body.point(point));
  }
}

template <size_t length>
LinearRotationalPoint SkidSteerCar<length>::reference() const
{
  return reference_;
}

template <size_t length>
void SkidSteerCar<length>::update(TimeUnit time)
{
  for (size_t i = 0; i < length; i++)
    wheels_on_body_[i].wheel.update(time);
}

template <size_t length>
void SkidSteerCar<length>::reset()
{
  for (size_t i = 0; i < length; i++)
    wheels_on_body_[i].wheel.reset();
}

}

#endif
