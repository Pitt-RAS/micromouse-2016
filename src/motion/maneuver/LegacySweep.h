#ifndef LEGACY_SWEEP_H
#define LEGACY_SWEEP_H

#include "Maneuver.h"

namespace Motion {

class LegacySweep : public Maneuver
{
  public:
    LegacySweep(AngleUnit angle, double size_scaling = 1.0);

    virtual void run();

  private:
    const AngleUnit angle_;
    const double size_scaling_;
};

}

#endif
