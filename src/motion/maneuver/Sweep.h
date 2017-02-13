#include "Maneuver.h"

namespace Motion {

class Sweep : public Maneuver
{
  public:
    struct Angle {
      enum { k45, k90, k135, k180 } magnitude;
      enum { left, right } direction;
    };

    Sweep(Angle angle);

    virtual void run();

  private:
    const Angle angle_;
};

}
