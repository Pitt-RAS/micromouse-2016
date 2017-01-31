#include "PointOnBody.h"

namespace Motion {

PointOnBody::PointOnBody(LengthUnit x_location, LengthUnit y_location) :
  kMetersPerRadian(x_location.meters()) // ignoring y_location
{}

LinearPoint PointOnBody::point(LinearRotationalPoint body_point)
{
  LinearPoint &linear = body_point.linear_point;
  RotationalPoint &rotational = body_point.rotational_point;

  double displacement = linear.displacement.meters();
  double     velocity = linear.velocity.meters();
  double acceleration = linear.acceleration.meters();

  displacement += kMetersPerRadian * rotational.displacement.radians();
      velocity += kMetersPerRadian * rotational.velocity.radians();
  acceleration += kMetersPerRadian * rotational.acceleration.radians();

  return {
    LengthUnit::fromMeters(displacement),
    LengthUnit::fromMeters(velocity),
    LengthUnit::fromMeters(acceleration)
  };
}

}
