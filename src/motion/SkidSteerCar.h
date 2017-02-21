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
    SkidSteerCar(WheelOnBody wheels_on_body[]);

    void reference(LinearRotationalPoint point);
    LinearRotationalPoint reference() const;

    virtual void update(TimeUnit time);

    void transition();

  protected:
    WheelOnBody *wheels_on_body_;

  private:
    LinearRotationalPoint reference_ = LinearRotationalPoint::zero();
};

template <size_t length>
class StraightenedSkidSteerCar : public SkidSteerCar<length>
{
  public:
    StraightenedSkidSteerCar(WheelOnBody wheels_on_body[]);

    virtual void update(TimeUnit time);

  private:
    AngleUnit wheelRotation();
};

template <size_t length>
SkidSteerCar<length>::SkidSteerCar(WheelOnBody wheels_on_body[]) :
  wheels_on_body_(wheels_on_body)
{}

template <size_t length>
void SkidSteerCar<length>::reference(LinearRotationalPoint point)
{
  reference_ = point;

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
void SkidSteerCar<length>::transition()
{
  for (size_t i = 0; i < length; i++)
    wheels_on_body_[i].wheel.transition();
}

template <size_t length>
StraightenedSkidSteerCar<length>::StraightenedSkidSteerCar(
                                                WheelOnBody wheels_on_body[]) :
  SkidSteerCar<length>(wheels_on_body)
{}

template <size_t length>
void StraightenedSkidSteerCar<length>::update(TimeUnit time)
{
  LinearRotationalPoint saved_reference = SkidSteerCar<length>::reference();
  LinearRotationalPoint straightened_reference = saved_reference;

  AngleUnit displacement = straightened_reference.rotational_point.displacement;
  double displacement_abstract = displacement.abstract();

  displacement_abstract += wheelRotation().abstract();

  displacement = AngleUnit::fromAbstract(displacement_abstract);
  straightened_reference.rotational_point.displacement = displacement;

  SkidSteerCar<length>::reference(straightened_reference);
  SkidSteerCar<length>::update(time);
  SkidSteerCar<length>::reference(saved_reference);
}

template <size_t length>
AngleUnit StraightenedSkidSteerCar<length>::wheelRotation()
{
  AngleUnit sum = AngleUnit::zero();

  for (int i = 0; i < length; i++) {
    Wheel &wheel = this->wheels_on_body_[i].wheel;
    PointOnBody &point_on_body = this->wheels_on_body_[i].point_on_body;

    AngleUnit angle = point_on_body.angle(wheel.displacement());
    sum = AngleUnit::fromAbstract(sum.abstract() + angle.abstract());
  }

  AngleUnit average = AngleUnit::fromAbstract(sum.abstract() / length);

  return average;
}

}

#endif
