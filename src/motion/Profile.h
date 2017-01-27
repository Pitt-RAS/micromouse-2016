#ifndef SPATIAL_PROFILE_H
#define SPATIAL_PROFILE_H

#include "units.h"

namespace Motion {

template <typename UnitType>
struct ProfilePoint
{
  UnitType displacement;
  UnitType velocity;
  UnitType acceleration;
};

template <typename PointType>
class Profile
{
  public:
    virtual PointType pointAtTime(TimeUnit time) = 0;
};

typedef ProfilePoint<LengthUnit> LinearPoint;
typedef ProfilePoint<AngleUnit> RotationalPoint;

struct LinearRotationalPoint
{
  LinearPoint linear_point;
  RotationalPoint rotational_point;
};

typedef Profile<LinearPoint> LinearProfile;
typedef Profile<RotationalPoint> RotationalProfile;
typedef Profile<LinearRotationalPoint> LinearRotationalProfile;

}

#endif
