#ifndef BODY_WITH_POINT_H
#define BODY_WITH_POINT_H

#include <cstdlib>
#include "units.h"
#include "Profile.h"

namespace Motion {

class BodyPoint
{
  public:
    BodyPoint(AngleUnit angle, LengthUnit radius);
    BodyPoint(LengthUnit x, LengthUnit y);
};

class BodyWithPoint
{
  public:
    BodyWithPoint(BodyPoint point);

    LinearProfile &pointProfile(LinearRotationalProfile body_profile);
};

}

#endif
