#ifndef PID_MATRIX_H
#define PID_MATRIX_H

#include "../PIDFunction.h"
#include "Matrix.h"

namespace Motion {

class PIDMatrix : public Matrix<PIDFunction>
{
  public:
    PIDMatrix(PIDParameters parameters);
};

}

#endif
