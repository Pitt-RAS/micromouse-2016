#ifndef TRAPEZOIDAL_PROFILE_H
#define TRAPEZOIDAL_PROFILE_H

#include "../legacy_motion/MotionCalc.h"
#include "Profile.h"

namespace Motion {

struct TrapezoidalConstraints
{
  LengthUnit distance;

  LengthUnit initial_velocity;
  LengthUnit final_velocity;

  LengthUnit max_velocity;

  LengthUnit acceleration;
  LengthUnit deceleration;
};

class TrapezoidalProfile : public LinearProfile
{
  public:
    TrapezoidalProfile(TrapezoidalConstraints constraints);

    virtual LinearPoint pointAtTime(TimeUnit time);

  private:
    MotionCalc legacy_implementation_;
};

}

#endif
