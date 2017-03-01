#include <Arduino.h>
#include "../../device/Encoder.h"
#include "../../device/Motor.h"
#include "../../device/RangeSensorContainer.h"
#include "../Matrix.h"
#include "../PIDFunction.h"
#include "Face.h"

namespace Motion {

const LengthUnit Face::kDistance = LengthUnit::fromMeters(0.022);
const TimeUnit Face::kTime = TimeUnit::fromSeconds(0.5);
const double Face::kVoltageLimit = 0.2;

void Face::run()
{
  PIDParameters parameters = { 0.0, 0.0, 0.0 };

  Matrix<PIDFunction> pid = Matrix<PIDFunction>(
    PIDFunction(parameters), PIDFunction(parameters),
    PIDFunction(parameters), PIDFunction(parameters)
  );

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

    gMotor.forEachWith<double>(
      voltage,
      [] (Motor &motor, double &voltage) -> void {
        motor.voltage(voltage);
      }
    );

    time = TimeUnit::fromSeconds(timer / 1e6);
  }

  gMotor.forEach(
    [] (Motor &motor) -> void {
      motor.voltage(0.0);
    }
  );

  gEncoder.forEach(
    [] (Encoder &encoder) -> void {
      encoder.displacement(LengthUnit::zero());
    }
  );
}

}
