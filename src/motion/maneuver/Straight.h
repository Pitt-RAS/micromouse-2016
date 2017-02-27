#include "Maneuver.h"

namespace Motion {

class Straight : public Maneuver
{
  public:
    Straight(LengthUnit length);

    virtual void run();

  protected:
    Straight(LengthUnit length,
                          bool zero_final_velocity, bool diagonal_correction);

  private:
    const LengthUnit length_;
    const bool zero_final_velocity_;
    const bool diagonal_correction_;
};

class Stop : public Straight
{
  public:
    Stop(LengthUnit length);
};

class Diagonal : public Straight
{
  public:
    Diagonal(LengthUnit length);
};

}
