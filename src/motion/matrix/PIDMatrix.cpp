#include "PIDMatrix.h"

namespace Motion {

PIDMatrix::PIDMatrix(PIDParameters parameters) :
  Matrix<PIDFunction>(
    PIDFunction(parameters), PIDFunction(parameters),
    PIDFunction(parameters), PIDFunction(parameters)
  )
{}

}
