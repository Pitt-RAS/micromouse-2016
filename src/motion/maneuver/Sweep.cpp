#include "../../legacy_motion/SweptTurnProfile.h"
#include "../../user_interaction/FreakOut.h"
#include "../tracker/Tracker.h"
#include "Sweep.h"

namespace Motion {

namespace {

  class LocalProfile : public LinearRotationalProfile
  {
    public:
      LocalProfile(Sweep::Angle angle, LengthUnit velocity);

      virtual LinearRotationalPoint pointAtTime(TimeUnit time);
      virtual TimeUnit finalTime();

    private:
      const SweptTurnProfile legacy_implementation_;

      const Sweep::Angle angle_;
      LengthUnit radius_ = LengthUnit::fromCells(0.5); // is this right?
      LengthUnit velocity_;

      double constructorTangentialVelocity(Sweep::Angle angle);
      double constructorTurnAngle(Sweep::Angle angle);
  };

  LocalProfile::LocalProfile(Sweep::Angle angle, LengthUnit velocity) :
    legacy_implementation_(constructorTangentialVelocity(angle),
                                                constructorTurnAngle(angle)),
    angle_(angle), velocity_(velocity)
  {}

  LinearRotationalPoint LocalProfile::pointAtTime(TimeUnit time)
  {
    LinearPoint linear_component = {
      LengthUnit::fromMeters(velocity_.meters() * time.seconds()),
      velocity_,
      LengthUnit::zero()
    };

    double seconds = time.seconds();

    double displacement = legacy_implementation_.getAngle(seconds);
    double     velocity = legacy_implementation_.getAngularVelocity(seconds);
    double acceleration =
                        legacy_implementation_.getAngularAcceleration(seconds);

    if (angle_.direction == Sweep::Angle::right) { // is this right??
      displacement = -displacement;
      velocity     = -velocity;
      acceleration = -acceleration;
    }

    RotationalPoint rotational_component = {
      AngleUnit::fromDegrees(displacement),
      AngleUnit::fromDegrees(velocity),
      AngleUnit::fromDegrees(acceleration)
    };

    return { linear_component, rotational_component };
  }

  TimeUnit LocalProfile::finalTime()
  {
    return TimeUnit::fromSeconds(legacy_implementation_.getTotalTime() / 1e6);
  }

  double constructorTangentialVelocity(Sweep::Angle angle)
  {
    switch (angle.magnitude) {
      case Sweep::Angle::k45:  return 0.8700;
      case Sweep::Angle::k90:  return 0.8400;
      case Sweep::Angle::k135: return 0.8975;
      case Sweep::Angle::k180: return 0.9350;
      default:                 return 0.0000;
    }
  }

  double constructorTurnAngle(Sweep::Angle angle)
  {
    switch (angle.magnitude) {
      case Sweep::Angle::k45:  return  45.0;
      case Sweep::Angle::k90:  return  90.0;
      case Sweep::Angle::k135: return 135.0;
      case Sweep::Angle::k180: return 180.0;
      default:                 return   0.0;
    }
  }

}

Sweep::Sweep(Angle angle) : angle_(angle)
{}

void Sweep::run()
{
  if (transition().linear_velocity.abstract()
                                    != constraints().sweep_velocity.abstract())
    freakOut("SWPT"); // consider more verbose logging

  TrackerOptions options;

  options.linear_ffw_parameters     = { 0.0, 0.0 };
  options.rotational_ffw_parameters = { 0.0, 0.0 };

  options.encoder_pid_parameters = { 0.0, 0.0, 0.0 };
  options.gyro_pid_parameters    = { 0.0, 0.0, 0.0 };

  LocalProfile profile(angle_, constraints().sweep_velocity);


  transition({ constraints().sweep_velocity });

  Tracker(options, profile).run();
}

}
