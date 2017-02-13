#include "../../user_interaction/FreakOut.h"
#include "../TrapezoidalProfile.h"
#include "../drive.h"
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

Pivot::Pivot(Angle angle) : Pivot(toContinuousAngle(angle))
{}

Pivot::Pivot(AngleUnit angle) : angle_(angle)
{}

void Pivot::run()
{
  if (transition().linear_velocity.abstract() != LengthUnit::zero().abstract())
    freakOut("PIVT"); // consider more verbose logging

  DriveOptions options;

  options.tuning = DriveOptions::kPivot;
  options.use_range = false;
  options.use_gyro = true;

  TrapezoidalConstraints<AngleUnit> trapezoidal_constraints;

  trapezoidal_constraints.distance = angle_;
  trapezoidal_constraints.initial_velocity = Transition::rotational_velocity;
  trapezoidal_constraints.final_velocity = Transition::rotational_velocity;
  trapezoidal_constraints.max_velocity = constraints().max_rotational_velocity;
  trapezoidal_constraints.acceleration = constraints().rotational_acceleration;
  trapezoidal_constraints.deceleration = constraints().rotational_deceleration;

  TrapezoidalProfile<AngleUnit> trapezoidal_profile(trapezoidal_constraints);
  LocalProfile profile(trapezoidal_profile);

  drive(options, profile);

  transition({ LengthUnit::zero() });
}

AngleUnit Pivot::toContinuousAngle(Angle discrete_angle)
{
  double magnitude = 0.0;

  switch (discrete_angle.magnitude) {
    case Angle::k45:  magnitude =  45.0; break;
    case Angle::k90:  magnitude =  90.0; break;
    case Angle::k180: magnitude = 180.0; break;
  }

  double result = 0.0;

  switch (discrete_angle.direction) {
    case Angle::left:  result =   magnitude; break;
    case Angle::right: result = - magnitude; break;
  }

  return AngleUnit::fromDegrees(result);
}

}
