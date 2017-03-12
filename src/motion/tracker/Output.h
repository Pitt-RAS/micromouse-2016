#ifndef OUTPUT_H
#define OUTPUT_H

#include "../Profile.h"
#include "../matrix/Matrix.h"

namespace Motion {

class Output
{
  public:
    virtual Matrix<double> output(LinearRotationalPoint target) = 0;
};

}

#endif
