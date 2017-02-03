#include "../user_interaction/FreakOut.h"
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
  trapezoidal_constraints.acceleration = constraints().linear_acceleration;
  trapezoidal_constraints.deceleration = constraints().linear_deceleration;

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
  Pivot::Profile profile(trapezoidal_profile);

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

Pivot::Profile::Profile(TrapezoidalProfile<AngleUnit> rotational_component) :
  rotational_component_(rotational_component)
{}

LinearRotationalPoint Pivot::Profile::pointAtTime(TimeUnit time)
{
  return {
    LinearPoint::zero(),
    rotational_component_.pointAtTime(time)
  };
}

}
