#ifndef POINT_ON_BODY_H
#define POINT_ON_BODY_H

#include <cstdlib>
#include "units.h"
#include "Profile.h"

namespace Motion {

class PointOnBody
{
  public:
    PointOnBody(LengthUnit x_location, LengthUnit y_location);

    LinearPoint point(LinearRotationalPoint body_point);

    AngleUnit angle(LengthUnit linear_displacement);

  private:
    const double kMetersPerRadian;
};

}

#endif
