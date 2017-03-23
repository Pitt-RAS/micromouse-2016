#ifndef PLANT_H
#define PLANT_H

#include "Maneuver.h"

namespace Motion {

class Plant : public Maneuver
{
  public:
    virtual void run();
};

}

#endif
