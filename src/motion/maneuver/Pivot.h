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
    static const AngleUnit kMaxVelocity;
    static const AngleUnit kAcceleration;
    static const AngleUnit kDeceleration;

    AngleUnit shortAngle() const;

    const AngleUnit angle_;
};

}
