#ifndef FACE_H
#define FACE_H

#include "Maneuver.h"

namespace Motion {

class Plant : public Maneuver
{
  public:
    virtual void run();
};

}

#endif
