#ifndef PID_FUNCTION_H
#define PID_FUNCTION_H

#include "../legacy_motion/PIDController.h"

namespace Motion {

struct PIDParameters
{
  double kp;
  double ki;
  double kd;
};

class PIDFunction
{
  public:
    PIDFunction(PIDParameters parameters);

    double response(double current, double setpoint);

    void reset();

  private:
    PIDParameters parameters_;
    PIDController legacy_implementation_;
};

}

#endif
