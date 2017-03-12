#ifndef OUTPUT_H
#define OUTPUT_H

#include "../Matrix.h"
#include "../Profile.h"

namespace Motion {

class Output
{
  public:
    virtual Matrix<double> output(LinearRotationalPoint target) = 0;
};

}

#endif
