#include "Wheel.h"

namespace Motion {

Wheel::Wheel(WheelOptions options, Motor &motor, Encoder &encoder) :
  motor_(motor), encoder_(encoder), pid_(options.pid_parameters)
{}

LengthUnit Wheel::displacement()
{
  return encoder_.displacement();
}

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
  double current_displacement = encoder_.displacement().meters();
  double target_displacement  = reference_.displacement.meters();

  double correction = pid_.response(current_displacement, target_displacement);

  double target_acceleration = reference_.acceleration.meters();
  double at_velocity = reference_.velocity.meters();

  motor_.Set(target_acceleration + correction, at_velocity);
}

void Wheel::transition()
{
  encoder_.displacement(LengthUnit::zero());

  double acceleration = reference_.acceleration.meters();
  double velocity = reference_.velocity.meters();

  motor_.Set(acceleration, velocity);
}

}
