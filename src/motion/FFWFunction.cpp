#include "FFWFunction.h"

namespace Motion {

FFWFunction::FFWFunction(FFWParameters parameters) : parameters_(parameters)
{}

double FFWFunction::output(double acceleration, double velocity)
{
  return parameters_.ka * acceleration + parameters_.kv * velocity;
}

}
