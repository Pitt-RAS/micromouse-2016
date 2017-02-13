#include "Wheel.h"

namespace Motion {

Wheel::Wheel(WheelOptions options, Motor &motor, Encoder &encoder) :
  motor_(motor), encoder_(encoder)
{}

void Wheel::reference(LinearPoint point)
{
  reference_ = point;
}

LinearPoint Wheel::reference() const
{
  return reference_;
}

void Wheel::update(TimeUnit time)
{
  // control
}

void Wheel::reset()
{
  reference_ = { LengthUnit::zero(), LengthUnit::zero(), LengthUnit::zero() };

  // reset control
}

}
