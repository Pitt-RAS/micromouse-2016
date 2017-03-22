#include "Maneuver.h"

namespace Motion {

class Sweep : public Maneuver
{
  public:
    Sweep(AngleUnit angle);

    virtual void run();

  private:
    const AngleUnit angle_;
};

}
