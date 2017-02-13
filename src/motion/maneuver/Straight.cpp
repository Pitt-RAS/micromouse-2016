#include "../TrapezoidalProfile.h"
#include "../drive.h"
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
  LocalProfile profile(trapezoidal_profile);

  drive(options, profile);

  transition({ trapezoidal_constraints.final_velocity });
}

Straight::Straight(LengthUnit length, bool zero_final_velocity) :
  length_(length), zero_final_velocity_(zero_final_velocity)
{}

Start::Start() : Straight(LengthUnit::fromCells(0.5))
{}

Stop::Stop() : Straight(LengthUnit::fromCells(0.5), true)
{}

}
