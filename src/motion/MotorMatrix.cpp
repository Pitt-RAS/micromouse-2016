#include "MotorMatrix.h"

namespace Motion {

MotorMatrix gMotor(
  gMotorLF, gMotorRF,
  gMotorLB, gMotorRB
);

MotorMatrix::MotorMatrix(Motor &a, Motor &b, Motor &c, Motor &d) :
  Matrix<Motor&>(a, b, c, d)
{}

void MotorMatrix::voltage(Matrix<double> value)
{
  forEachWith<double>(
    value,
    [] (Motor &motor, double &value) -> void {
      motor.voltage(value);
    }
  );
}

void MotorMatrix::zero()
{
  forEach(
    [] (Motor &motor) -> void {
      motor.voltage(0.0);
    }
  );
}

}
