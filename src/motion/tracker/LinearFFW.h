#ifndef LINEAR_FFW_H
#define LINEAR_FFW_H

#include "../FFWFunction.h"
#include "Output.h"

namespace Motion {

class LinearFFW : public Output
{
  public:
    LinearFFW(FFWParameters parameters);

    virtual Matrix<double> output(LinearRotationalPoint target);

  private:
    FFWFunction ffw_;
};

}

#endif
