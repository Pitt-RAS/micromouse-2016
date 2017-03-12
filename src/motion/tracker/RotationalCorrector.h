#ifndef ROTATIONAL_CORRECTOR_H
#define ROTATIONAL_CORRECTOR_H

#include "../PIDFunction.h"
#include "Output.h"

namespace Motion {

class RotationalCorrector : public StatefulOutput
{
  public:
    RotationalCorrector(PIDParameters gyro_pid_parameters,
                                          PIDParameters range_pid_parameters);

    virtual Matrix<double> output(LinearRotationalPoint target);

  private:
    PIDFunction gyro_pid_;
    PIDFunction range_pid_;
};

}

#endif
