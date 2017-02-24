#include "../TrapezoidalProfile.h"
#include "../tracker/Tracker.h"
#include "Straight.h"

namespace Motion {

namespace {

  class LocalProfile : public LinearRotationalProfile
  {
    public:
      LocalProfile(TrapezoidalProfile<LengthUnit> linear_component);

      virtual LinearRotationalPoint pointAtTime(TimeUnit time);
      virtual TimeUnit finalTime();

    private:
      TrapezoidalProfile<LengthUnit> linear_component_;
  };

  LocalProfile::LocalProfile(TrapezoidalProfile<LengthUnit> linear_component) :
    linear_component_(linear_component)
  {}

  LinearRotationalPoint LocalProfile::pointAtTime(TimeUnit time)
  {
    return {
      linear_component_.pointAtTime(time),
      RotationalPoint::zero()
    };
  }

  TimeUnit LocalProfile::finalTime()
  {
    return linear_component_.finalTime();
  }

}

Straight::Straight(LengthUnit length) : Straight(length, false, false)
{}

void Straight::run()
{
  TrackerOptions options;

  options.linear_ffw_parameters     = { 0.005, 0.26 };
  options.rotational_ffw_parameters = { 0.0, 0.0 };

  options.encoder_pid_parameters = { 100.0, 0.0, 0.0 };
  options.gyro_pid_parameters    = { 0.006, 0.0, 0.0 };
  options.range_pid_parameters   = { 0.001, 0.0, 0.0 };

  options.diagonal_correction = diagonal_correction_;

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
  LocalProfile profile(trapezoidal_profile);

  Tracker(options, profile).run();

  transition({ trapezoidal_constraints.final_velocity });
}

Straight::Straight(LengthUnit length,
                          bool zero_final_velocity, bool diagonal_correction) :
  length_(length),
  zero_final_velocity_(zero_final_velocity),
  diagonal_correction_(diagonal_correction)
{}

Stop::Stop(LengthUnit length) : Straight(length, true, false)
{}

Diagonal::Diagonal(LengthUnit length) : Straight(length, false, true)
{}

}
