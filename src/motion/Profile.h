#ifndef SPATIAL_PROFILE_H
#define SPATIAL_PROFILE_H

#include "units.h"

namespace Motion {

template <typename UnitType>
class ProfilePoint
{
  public:
    static ProfilePoint<UnitType> zero();

    UnitType displacement;
    UnitType velocity;
    UnitType acceleration;
};

template <typename PointType>
class Profile
{
  public:
    virtual PointType pointAtTime(TimeUnit time) = 0;
    virtual TimeUnit finalTime() = 0;
};

typedef ProfilePoint<LengthUnit> LinearPoint;
typedef ProfilePoint<AngleUnit> RotationalPoint;

class LinearRotationalPoint
{
  public:
    static LinearRotationalPoint zero();

    LinearPoint linear_point;
    RotationalPoint rotational_point;
};

typedef Profile<LinearPoint> LinearProfile;
typedef Profile<RotationalPoint> RotationalProfile;
typedef Profile<LinearRotationalPoint> LinearRotationalProfile;

}

#endif
