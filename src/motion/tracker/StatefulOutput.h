#ifndef STATEFUL_OUTPUT_H
#define STATEFUL_OUTPUT_H

#include "Output.h"

namespace Motion {

class StatefulOutput : public Output
{
  public:
    virtual void reset() = 0;
};

}

#endif
