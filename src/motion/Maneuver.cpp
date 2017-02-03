#include "../legacy_motion/SweptTurnProfile.h"
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

Straight::Straight(LengthUnit length) : Straight(length, false)
{}

void Straight::run()
{
  DriveOptions options;

  options.tuning = DriveOptions::kStraight;
  options.use_range = true;
  options.use_gyro = true;

  TrapezoidalConstraints<LengthUnit> trapezoidal_constraints;

  LengthUnit final_velocity = zero_final_velocity_ ?
                            LengthUnit::zero() : constraints().sweep_velocity;

  trapezoidal_constraints.distance = length_;
  trapezoidal_constraints.initial_velocity = transition().linear_velocity;
  trapezoidal_constraints.final_velocity = final_velocity;
  trapezoidal_constraints.max_velocity = constraints().max_forward_velocity;
  trapezoidal_constraints.acceleration = constraints().linear_acceleration;
  trapezoidal_constraints.deceleration = constraints().linear_deceleration;

  TrapezoidalProfile<LengthUnit> trapezoidal_profile(trapezoidal_constraints);
  Straight::Profile profile(trapezoidal_profile);

  drive(options, profile);

  transition({ trapezoidal_constraints.final_velocity });
}

Straight::Straight(LengthUnit length, bool zero_final_velocity) :
  length_(length), zero_final_velocity_(zero_final_velocity)
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

TimeUnit Straight::Profile::finalTime()
{
  return linear_component_.finalTime();
}

Start::Start() : Straight(LengthUnit::fromCells(0.5))
{}

Stop::Stop() : Straight(LengthUnit::fromCells(0.5), true)
{}

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

TimeUnit Pivot::Profile::finalTime()
{
  return rotational_component_.finalTime();
}

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
