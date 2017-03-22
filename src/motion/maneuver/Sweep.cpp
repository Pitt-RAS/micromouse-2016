#include "../../legacy_motion/SweptTurnProfile.h"
#include "../../user_interaction/FreakOut.h"
#include "../tracker/Tracker.h"
#include "Sweep.h"

// not sure why this is needed here
#include "../../undef.h"

namespace Motion {

namespace {

  class LocalProfile : public LinearRotationalProfile
  {
    public:
      LocalProfile(AngleUnit angle, LengthUnit velocity);

      virtual LinearRotationalPoint pointAtTime(TimeUnit time);
      virtual TimeUnit finalTime();

    private:
      LengthUnit referenceVelocity();
      double referenceScaling();

      const SweptTurnProfile legacy_implementation_;

      const AngleUnit angle_;
      LengthUnit velocity_;
  };

  LocalProfile::LocalProfile(AngleUnit angle, LengthUnit velocity) :
    legacy_implementation_(velocity.meters(), std::fabs(angle.degrees())),
    angle_(angle), velocity_(velocity)
  {}

  LinearRotationalPoint LocalProfile::pointAtTime(TimeUnit time)
  {
    LinearPoint linear_component = {
      LengthUnit::fromMeters(velocity_.meters() * time.seconds()),
      velocity_,
      LengthUnit::zero()
    };

    double seconds = time.seconds() * referenceScaling();

    double displacement = legacy_implementation_.getAngle(seconds);
    double     velocity = legacy_implementation_.getAngularVelocity(seconds);
    double acceleration =
                      legacy_implementation_.getAngularAcceleration(seconds);

    if (angle_.abstract() < 0) {
      displacement = -displacement;
      velocity     = -velocity;
      acceleration = -acceleration;
    }

    double pi = 3.14159265359;

    RotationalPoint rotational_component = {
      AngleUnit::fromRadians(displacement),
      AngleUnit::fromRadians(velocity / pi), // hacky approximation
      AngleUnit::fromRadians(acceleration / pi / pi) // hacky approximation
    };

    return { linear_component, rotational_component };
  }

  TimeUnit LocalProfile::finalTime()
  {
    double unscaled = legacy_implementation_.getTotalTime();

    return TimeUnit::fromSeconds(unscaled / referenceScaling());
  }

  LengthUnit LocalProfile::referenceVelocity()
  {
    int int_angle = (int) std::fabs(angle_.degrees());

    switch (int_angle) {
      default:  return LengthUnit::fromMeters(0.0000);
      case  45: return LengthUnit::fromMeters(0.8700);
      case  90: return LengthUnit::fromMeters(0.8400);
      case 135: return LengthUnit::fromMeters(0.8975);
      case 180: return LengthUnit::fromMeters(0.9350);
    }
  }

  double LocalProfile::referenceScaling()
  {
    double time_scaling = velocity_.abstract() / referenceVelocity().abstract();
    double size_scaling = 1.0;

    return time_scaling / size_scaling;
  }

}

Sweep::Sweep(AngleUnit angle) : angle_(angle)
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
