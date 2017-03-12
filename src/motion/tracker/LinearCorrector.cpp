#include "../matrix/EncoderMatrix.h"
#include "../matrix/PIDMatrix.h"
#include "LinearCorrector.h"

namespace Motion {

LinearCorrector::LinearCorrector(PIDParameters encoder_pid_parameters) :
  encoder_pid_(PIDMatrix(encoder_pid_parameters))
{}

Matrix<double> LinearCorrector::output(LinearRotationalPoint target)
{
  LengthUnit scalar_setpoint = target.linear_point.displacement;

  Matrix<LengthUnit> current = gEncoder.linearDisplacement();
  Matrix<LengthUnit> setpoint = Matrix<LengthUnit>::splat(scalar_setpoint);

  return encoder_pid_.mapWith<double, LengthUnit, LengthUnit>(
    current,
    setpoint,
    [] (PIDFunction &pid, LengthUnit &current, LengthUnit &setpoint) -> double {
      return pid.response(current.meters(), setpoint.meters());
    }
  );
}

}
