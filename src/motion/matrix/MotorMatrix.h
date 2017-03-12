#ifndef MOTOR_MATRIX_H
#define MOTOR_MATRIX_H

#include "../../device/Motor.h"
#include "Matrix.h"

namespace Motion {

class MotorMatrix : public Matrix<Motor&>
{
  public:
    MotorMatrix(Motor &a, Motor &b, Motor &c, Motor &d);

    void voltage(Matrix<double> value);

    void zero();
};

extern MotorMatrix gMotor;

}

#endif
