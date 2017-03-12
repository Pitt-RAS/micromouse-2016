#ifndef ENCODER_MATRIX_H
#define ENCODER_MATRIX_H

#include "../device/Encoder.h"
#include "Matrix.h"

namespace Motion {

class EncoderMatrix : public Matrix<Encoder&>
{
  public:
    EncoderMatrix(Encoder &a, Encoder &b, Encoder &c, Encoder &d);

    void zero();

    Matrix<LengthUnit> linearDisplacement();
    LengthUnit averageDisplacement();

    void zeroLinearDisplacement(LengthUnit new_zero);
};

extern EncoderMatrix gEncoder;

}

#endif
