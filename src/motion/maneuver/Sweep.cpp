#include "../../legacy_motion/SweptTurnProfile.h"
#include "../../user_interaction/FreakOut.h"
#include "../drive.h"
#include "Sweep.h"

namespace Motion {

const SweptTurnProfile Sweep::Profile::kLegacyProfile45 (0.8700,  45.0);
const SweptTurnProfile Sweep::Profile::kLegacyProfile90 (0.8400,  90.0);
const SweptTurnProfile Sweep::Profile::kLegacyProfile135(0.8975, 135.0);
const SweptTurnProfile Sweep::Profile::kLegacyProfile180(0.9350, 180.0);

Sweep::Sweep(Angle angle) : angle_(angle)
{}

void Sweep::run()
{
  if (transition().linear_velocity.abstract()
                                    != constraints().sweep_velocity.abstract())
    freakOut("SWPT"); // consider more verbose logging

  DriveOptions options;

  options.tuning = DriveOptions::kSweep;
  options.use_range = false;
  options.use_gyro = true;

  Sweep::Profile profile(angle_, constraints().sweep_velocity);

  drive(options, profile);

  transition({ constraints().sweep_velocity });
}

Sweep::Profile::Profile(Angle angle, LengthUnit velocity) :
  legacy_implementation_(toLegacyImplementation(angle)), velocity_(velocity)
{}

LinearRotationalPoint Sweep::Profile::pointAtTime(TimeUnit time)
{
  LinearPoint linear_component = {
    LengthUnit::fromMeters(velocity_.meters() * time.seconds()),
    velocity_,
    LengthUnit::zero()
  };

  double seconds = time.seconds();

  double displacement = legacy_implementation_.getAngle(seconds);
  double     velocity = legacy_implementation_.getAngularVelocity(seconds);
  double acceleration = legacy_implementation_.getAngularAcceleration(seconds);

  RotationalPoint rotational_component = {
    AngleUnit::fromDegrees(displacement),
    AngleUnit::fromDegrees(velocity),
    AngleUnit::fromDegrees(acceleration)
  };

  return { linear_component, rotational_component };
}

TimeUnit Sweep::Profile::finalTime()
{
  return TimeUnit::fromSeconds(legacy_implementation_.getTotalTime() / 1e6);
}

const SweptTurnProfile &Sweep::Profile::toLegacyImplementation(Angle angle)
{
  switch (angle.magnitude) {
    case Angle::k45:  return kLegacyProfile45;
    case Angle::k90:  return kLegacyProfile90;
    case Angle::k135: return kLegacyProfile135;
    case Angle::k180: return kLegacyProfile180;
    default: return kLegacyProfile45;
  }
}

}
