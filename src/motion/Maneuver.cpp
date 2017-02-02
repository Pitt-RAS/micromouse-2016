#include "TrapezoidalProfile.h"
#include "drive.h"
#include "units.h"
#include "Maneuver.h"

namespace Motion {

const AngleUnit Transition::rotational_velocity = AngleUnit::zero();

ManeuverConstraints Maneuver::constraints()
{
  return constraints_;
}

void Maneuver::constraints(ManeuverConstraints constraints)
{
  constraints_ = constraints;
}

Transition Maneuver::transition()
{
  return transition_;
}

void Maneuver::transition(Transition transition)
{
  transition_ = transition;
}

Straight::Straight(LengthUnit length) :
  Straight(length, constraints().sweep_velocity)
{}

void Straight::run()
{
  DriveOptions options;

  options.tuning = DriveOptions::kStraight;
  options.use_range = true;
  options.use_gyro = true;

  TrapezoidalConstraints<LengthUnit> trapezoidal_constraints;

  trapezoidal_constraints.distance = length_;
  trapezoidal_constraints.initial_velocity = transition().linear_velocity;
  trapezoidal_constraints.final_velocity = final_velocity_;
  trapezoidal_constraints.max_velocity = constraints().max_forward_velocity;
  trapezoidal_constraints.acceleration = constraints().acceleration;
  trapezoidal_constraints.deceleration = constraints().deceleration;

  TrapezoidalProfile<LengthUnit> trapezoidal_profile(trapezoidal_constraints);
  Straight::Profile profile(trapezoidal_profile);

  drive(options, profile);

  transition({ trapezoidal_constraints.final_velocity });
}

Straight::Straight(LengthUnit length, LengthUnit final_velocity) :
  length_(length), final_velocity_(final_velocity)
{}

Straight::Profile::Profile(TrapezoidalProfile<LengthUnit> linear_component) :
  linear_component_(linear_component)
{}

LinearRotationalPoint Straight::Profile::pointAtTime(TimeUnit time)
{
  return {
    linear_component_.pointAtTime(time),
    RotationalPoint::zero()
  };
}

}
