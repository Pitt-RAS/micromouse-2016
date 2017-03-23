#include "../../user_interaction/FreakOut.h"
#include "../TrapezoidalProfile.h"
#include "../tracker/Tracker.h"
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
    // hack
    if (time.abstract() == finalTime().abstract()) {
      AngleUnit angle = rotational_component_.pointAtTime(time).displacement;
      angle = AngleUnit::fromAbstract(angle.abstract() / 10.0);

      return {
        LinearPoint::zero(),
        { angle, AngleUnit::zero(), AngleUnit::zero() }
      };
    }

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

const AngleUnit Pivot::kMaxVelocity = AngleUnit::fromRotations(0.0);
const AngleUnit Pivot::kAcceleration = AngleUnit::fromRotations(0.0);
const AngleUnit Pivot::kDeceleration = AngleUnit::fromRotations(0.0);

Pivot::Pivot(AngleUnit angle) : angle_(angle)
{}

void Pivot::run()
{
  if (transition().linear_velocity.abstract() != LengthUnit::zero().abstract())
    freakOut("PIVT"); // consider more verbose logging

  if (angle_.abstract() == 0)
    return;

  TrackerOptions options;

  options.linear_ffw_parameters     = { 0.0, 0.0 };
  options.rotational_ffw_parameters = { 0.0, 0.0 };

  options.encoder_pid_parameters = { 0.0, 0.0, 0.0 };

  options.end_condition = TrackerOptions::kGyroAngle;
  options.end_condition_data.angle = shortAngle();

  options.gyro_safety_check = false;

  TrapezoidalConstraints<AngleUnit> trapezoidal_constraints;

  trapezoidal_constraints.distance = angle_;
  trapezoidal_constraints.initial_velocity = Transition::rotational_velocity;
  trapezoidal_constraints.final_velocity = Transition::rotational_velocity;
  trapezoidal_constraints.max_velocity = kMaxVelocity;
  trapezoidal_constraints.acceleration = kAcceleration;
  trapezoidal_constraints.deceleration = kDeceleration;

  // hack
  AngleUnit distance = trapezoidal_constraints.distance;
  distance = AngleUnit::fromAbstract(10.0 * distance.abstract());
  trapezoidal_constraints.distance = distance;

  TrapezoidalProfile<AngleUnit> trapezoidal_profile(trapezoidal_constraints);
  LocalProfile profile(trapezoidal_profile);

  transition({ LengthUnit::zero() });

  Tracker(options, profile).run();
}

AngleUnit Pivot::shortAngle() const
{
  AngleUnit offset = AngleUnit::fromDegrees(5.0);

  if (angle_.abstract() < 0.0)
    offset = AngleUnit::fromAbstract(-offset.abstract());

  return AngleUnit::fromAbstract(angle_.abstract() - offset.abstract());
}

}
