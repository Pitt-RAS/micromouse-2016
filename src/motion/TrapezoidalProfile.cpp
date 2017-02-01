#include "TrapezoidalProfile.h"

namespace Motion {

TrapezoidalProfile::TrapezoidalProfile(TrapezoidalConstraints constraints) :
  legacy_implementation_(
    constraints.distance.meters() * 1000.0,
    constraints.max_velocity.meters() * 1000.0,
    constraints.initial_velocity.meters() * 1000.0,
    constraints.final_velocity.meters() * 1000.0,
    constraints.acceleration.meters() * 1000.0,
    constraints.deceleration.meters() * 1000.0
  )
{}

LinearPoint TrapezoidalProfile::pointAtTime(TimeUnit time)
{
  double seconds = time.seconds();

  double displacement = legacy_implementation_.idealDistance(seconds);
  double     velocity = legacy_implementation_.idealVelocity(seconds);
  double acceleration = legacy_implementation_.idealAccel(seconds);

  return {
    LengthUnit::fromMeters(displacement / 1000.0),
    LengthUnit::fromMeters(velocity / 1000.0),
    LengthUnit::fromMeters(acceleration / 1000.0)
  };
}

}
