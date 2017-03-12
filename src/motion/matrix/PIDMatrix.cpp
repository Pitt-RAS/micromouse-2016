#include "PIDMatrix.h"

namespace Motion {

PIDMatrix::PIDMatrix(PIDParameters parameters) :
  Matrix<PIDFunction>(
    PIDFunction(parameters), PIDFunction(parameters),
    PIDFunction(parameters), PIDFunction(parameters)
  ),
  parameters_(parameters)
{}

void PIDMatrix::reset()
{
  *this = PIDMatrix(parameters_);
}

}
