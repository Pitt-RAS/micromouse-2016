#ifndef LINEAR_CORRECTOR_H
#define LINEAR_CORRECTOR_H

#include "../matrix/PIDMatrix.h"
#include "StatefulOutput.h"

namespace Motion {

class LinearCorrector : public StatefulOutput
{
  public:
    LinearCorrector(PIDParameters encoder_pid_parameters);

    virtual Matrix<double> output(LinearRotationalPoint target);

    virtual void reset();

  private:
    PIDMatrix encoder_pid_;
};

}

#endif
