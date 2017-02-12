#include "Maneuver.h"

namespace Motion {

class Straight : public Maneuver
{
  public:
    Straight(LengthUnit length);

    virtual void run();

  protected:
    Straight(LengthUnit length, bool zero_final_velocity);

  private:
    class Profile : public LinearRotationalProfile
    {
      public:
        Profile(TrapezoidalProfile<LengthUnit> linear_component);

        virtual LinearRotationalPoint pointAtTime(TimeUnit time);
        virtual TimeUnit finalTime();

      private:
        TrapezoidalProfile<LengthUnit> linear_component_;
    };

    const LengthUnit length_;
    const bool zero_final_velocity_;
};

class Start : public Straight
{
  public:
    Start();
};

class Stop : public Straight
{
  public:
    Stop();
};

}
