#include "../../device/Orientation.h"
#include "RotationalCorrector.h"

namespace Motion {

RotationalCorrector::RotationalCorrector(PIDParameters gyro_pid_parameters,
                                          PIDParameters range_pid_parameters) :
  gyro_pid_(gyro_pid_parameters), range_pid_(range_pid_parameters)
{}

Matrix<double> RotationalCorrector::output(LinearRotationalPoint target)
{
  gOrientation->update();

  AngleUnit current = AngleUnit::fromDegrees(-gOrientation->getHeading());
  AngleUnit setpoint = target.rotational_point.displacement;

  double response = gyro_pid_.response(current.degrees(), setpoint.degrees());

  return Matrix<double>::splat(response).oppose();
}

void RotationalCorrector::reset()
{
  gyro_pid_.reset();
  range_pid_.reset();
}

}
