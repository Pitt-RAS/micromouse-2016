#include "Maneuver.h"

namespace Motion {

class Pivot : public Maneuver
{
  public:
    class Angle {
      public:
        enum { k45, k90, k180 } magnitude;
        enum { left, right } direction;

        AngleUnit toContinuous();
    };

    Pivot(Angle angle);
    Pivot(AngleUnit angle);

    virtual void run();

  private:
    const AngleUnit angle_;
};

}
