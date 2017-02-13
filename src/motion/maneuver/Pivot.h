#include "Maneuver.h"

namespace Motion {

class Pivot : public Maneuver
{
  public:
    struct Angle {
      enum { k45, k90, k180 } magnitude;
      enum { left, right } direction;
    };

    Pivot(Angle angle);
    Pivot(AngleUnit angle);

    virtual void run();

  private:
    const AngleUnit angle_;

    AngleUnit toContinuousAngle(Angle discrete_angle);
};

}
