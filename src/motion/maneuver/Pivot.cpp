#include "../../user_interaction/FreakOut.h"
#include "../Tracker.h"
#include "../TrapezoidalProfile.h"
#include "Pivot.h"

namespace Motion {

namespace {

  class LocalProfile : public LinearRotationalProfile
  {
    public:
      LocalProfile(TrapezoidalProfile<AngleUnit> rotational_component);

      virtual LinearRotationalPoint pointAtTime(TimeUnit time);
      virtual TimeUnit finalTime();

    private:
      TrapezoidalProfile<AngleUnit> rotational_component_;
  };

  LocalProfile::LocalProfile(
                        TrapezoidalProfile<AngleUnit> rotational_component) :
    rotational_component_(rotational_component)
  {}

  LinearRotationalPoint LocalProfile::pointAtTime(TimeUnit time)
  {
    return {
      LinearPoint::zero(),
      rotational_component_.pointAtTime(time)
    };
  }

  TimeUnit LocalProfile::finalTime()
  {
    return rotational_component_.finalTime();
  }

}

AngleUnit Pivot::Angle::toContinuous()
{
  double result = 0.0;

  switch (magnitude) {
    case Angle::k45:  result =  45.0; break;
    case Angle::k90:  result =  90.0; break;
    case Angle::k180: result = 180.0; break;
  }

  switch (direction) {
    case Angle::left:  result =   magnitude; break;
    case Angle::right: result = - magnitude; break;
  }

  return AngleUnit::fromDegrees(result);
}

Pivot::Pivot(Angle angle) : Pivot(angle.toContinuous())
{}

Pivot::Pivot(AngleUnit angle) : angle_(angle)
{}

void Pivot::run()
{
  if (transition().linear_velocity.abstract() != LengthUnit::zero().abstract())
    freakOut("PIVT"); // consider more verbose logging

  TrackerOptions options;

  options.linear_ffw_parameters     = { 0.0, 0.0 };
  options.rotational_ffw_parameters = { 0.0, 0.0 };

  options.encoder_pid_parameters = { 0.0, 0.0, 0.0 };
  options.gyro_pid_parameters    = { 0.0, 0.0, 0.0 };

  TrapezoidalConstraints<AngleUnit> trapezoidal_constraints;

  trapezoidal_constraints.distance = angle_;
  trapezoidal_constraints.initial_velocity = Transition::rotational_velocity;
  trapezoidal_constraints.final_velocity = Transition::rotational_velocity;
  trapezoidal_constraints.max_velocity = constraints().max_rotational_velocity;
  trapezoidal_constraints.acceleration = constraints().rotational_acceleration;
  trapezoidal_constraints.deceleration = constraints().rotational_deceleration;

  TrapezoidalProfile<AngleUnit> trapezoidal_profile(trapezoidal_constraints);
  LocalProfile profile(trapezoidal_profile);

  Tracker(options, profile).run();

  transition({ LengthUnit::zero() });
}

}
