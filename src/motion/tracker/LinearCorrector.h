#ifndef LINEAR_CORRECTOR_H
#define LINEAR_CORRECTOR_H

#include "../PIDFunction.h"
#include "Output.h"

namespace Motion {

class LinearCorrector : public Output
{
  public:
    LinearCorrector(PIDParameters encoder_pid_parameters);

    virtual Matrix<double> output(LinearRotationalPoint target);

  private:
    Matrix<PIDFunction> encoder_pid_;
};

}

#endif
