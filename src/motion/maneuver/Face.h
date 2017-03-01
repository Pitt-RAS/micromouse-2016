#ifndef FACE_H
#define FACE_H

#include "Maneuver.h"

namespace Motion {

class Face : public Maneuver
{
  public:
    virtual void run();

  private:
    static const LengthUnit kDistance;
    static const TimeUnit kTime;
    static const double kVoltageLimit;
};

}

#endif
