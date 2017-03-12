#include <Arduino.h>
#include "../../device/RangeSensorContainer.h"
#include "../matrix/EncoderMatrix.h"
#include "../matrix/Matrix.h"
#include "../matrix/MotorMatrix.h"
#include "../matrix/PIDMatrix.h"
#include "Face.h"

namespace Motion {

const LengthUnit Face::kDistance = LengthUnit::fromMeters(0.022);
const TimeUnit Face::kTime = TimeUnit::fromSeconds(0.5);
const double Face::kVoltageLimit = 0.2;

void Face::run()
{
  PIDMatrix pid({ 0.0, 0.0, 0.0 });

  elapsedMicros timer;

  TimeUnit time = TimeUnit::fromSeconds(timer / 1e6);

  while (time.abstract() < kTime.abstract()) {
    double left = RangeSensors.frontLeftSensor.getRange();
    double right = RangeSensors.frontRightSensor.getRange();

    Matrix<double> current = Matrix<double>(
      left, right,
      left, right
    );

    Matrix<double> setpoint = Matrix<double>::ones().multiply(
      kDistance.meters()
    );

    Matrix<double> voltage = pid.mapWith<double, double, double>(
      current,
      setpoint,
      [] (PIDFunction &pid, double &current, double &setpoint) -> double {
        return pid.response(current, setpoint);
      }
    );

    voltage = voltage.map<double>(
      [] (double &voltage) -> double {
        if (voltage < -kVoltageLimit) voltage = -kVoltageLimit;
        if (voltage >  kVoltageLimit) voltage =  kVoltageLimit;
        return voltage;
      }
    );

    gMotor.voltage(voltage);

    time = TimeUnit::fromSeconds(timer / 1e6);
  }

  gMotor.zero();
  gEncoder.zero();
}

}
