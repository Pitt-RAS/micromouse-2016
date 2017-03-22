#include "Maneuver.h"

namespace Motion {

class Pivot : public Maneuver
{
  public:
    Pivot(AngleUnit angle);

    virtual void run();

  private:
    static const AngleUnit kMaxVelocity;
    static const AngleUnit kAcceleration;
    static const AngleUnit kDeceleration;

    AngleUnit shortAngle() const;

    const AngleUnit angle_;
};

}
