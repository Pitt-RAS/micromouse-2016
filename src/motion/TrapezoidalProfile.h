#ifndef TRAPEZOIDAL_PROFILE_H
#define TRAPEZOIDAL_PROFILE_H

#include "../legacy_motion/MotionCalc.h"
#include "Profile.h"

namespace Motion {

template <typename UnitType>
struct TrapezoidalConstraints
{
  UnitType distance = UnitType::zero();

  UnitType initial_velocity = UnitType::zero();
  UnitType final_velocity = UnitType::zero();

  UnitType max_velocity = UnitType::zero();

  UnitType acceleration = UnitType::zero();
  UnitType deceleration = UnitType::zero();
};

template <typename UnitType>
class TrapezoidalProfile : public Profile<ProfilePoint<UnitType>>
{
  public:
    TrapezoidalProfile(TrapezoidalConstraints<UnitType> constraints);

    virtual ProfilePoint<UnitType> pointAtTime(TimeUnit time);
    virtual TimeUnit finalTime();

  private:
    MotionCalc legacy_implementation_;
};

template <typename UnitType>
TrapezoidalProfile<UnitType>::TrapezoidalProfile(
                                TrapezoidalConstraints<UnitType> constraints) :
  legacy_implementation_(
    constraints.distance.abstract(),
    constraints.max_velocity.abstract(),
    constraints.initial_velocity.abstract(),
    constraints.final_velocity.abstract(),
    constraints.acceleration.abstract(),
    constraints.deceleration.abstract()
  )
{}

template <typename UnitType>
ProfilePoint<UnitType> TrapezoidalProfile<UnitType>::pointAtTime(TimeUnit time)
{
  double seconds = time.seconds();

  double displacement = legacy_implementation_.idealDistance(seconds);
  double     velocity = legacy_implementation_.idealVelocity(seconds);
  double acceleration = legacy_implementation_.idealAccel(seconds);

  return {
    UnitType::fromAbstract(displacement),
    UnitType::fromAbstract(velocity),
    UnitType::fromAbstract(acceleration)
  };
}

template <typename UnitType>
TimeUnit TrapezoidalProfile<UnitType>::finalTime()
{
  return TimeUnit::fromSeconds(legacy_implementation_.getTotalTime() / 1e6);
}

}

#endif
