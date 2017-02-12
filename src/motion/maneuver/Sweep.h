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
    class Profile : public LinearRotationalProfile
    {
      public:
        Profile(Angle angle, LengthUnit velocity);

        virtual LinearRotationalPoint pointAtTime(TimeUnit time);
        virtual TimeUnit finalTime();

      private:
        static const SweptTurnProfile kLegacyProfile45;
        static const SweptTurnProfile kLegacyProfile90;
        static const SweptTurnProfile kLegacyProfile135;
        static const SweptTurnProfile kLegacyProfile180;

        const SweptTurnProfile &legacy_implementation_;

        LengthUnit radius_ = LengthUnit::fromCells(0.5); // is this right?
        LengthUnit velocity_;

        const SweptTurnProfile &toLegacyImplementation(Angle angle);
    };

    Angle angle_;
};

}
