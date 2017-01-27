#ifndef TRAPEZOIDAL_PROFILE_H
#define TRAPEZOIDAL_PROFILE_H

#include "Profile.h"

namespace Motion {

class TrapezoidalProfile : public LinearProfile
{
  public:
    TrapezoidalProfile(TimeUnit time, LengthUnit max_velocity,
                        LengthUnit initial_velocity, LengthUnit final_velocity);

    virtual LinearPoint pointAtTime(TimeUnit time);
};

}

#endif
