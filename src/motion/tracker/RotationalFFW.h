#ifndef ROTATIONAL_FFW_H
#define ROTATIONAL_FFW_H

#include "../FFWFunction.h"
#include "Output.h"

namespace Motion {

class RotationalFFW : public Output
{
  public:
    RotationalFFW(FFWParameters parameters);

    virtual Matrix<double> output(LinearRotationalPoint target);

  private:
    FFWFunction ffw_;
};

}

#endif
