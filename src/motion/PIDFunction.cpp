#include "PIDFunction.h"

namespace Motion {

PIDFunction::PIDFunction(PIDParameters parameters) :
  legacy_implementation_(parameters.kp, parameters.ki, parameters.kd)
{}

double PIDFunction::response(double current, double setpoint)
{
  return legacy_implementation_.Calculate(current, setpoint);
}

}
