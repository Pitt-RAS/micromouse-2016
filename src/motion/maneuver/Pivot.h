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
    class Profile : public LinearRotationalProfile
    {
      public:
        Profile(TrapezoidalProfile<AngleUnit> rotational_component);

        virtual LinearRotationalPoint pointAtTime(TimeUnit time);
        virtual TimeUnit finalTime();

      private:
        TrapezoidalProfile<AngleUnit> rotational_component_;
    };

    AngleUnit toContinuousAngle(Angle discrete_angle);

    const AngleUnit angle_;
};

}
