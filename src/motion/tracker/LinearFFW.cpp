#include "LinearFFW.h"

namespace Motion {

LinearFFW::LinearFFW(FFWParameters parameters) : ffw_(parameters)
{}

Matrix<double> LinearFFW::output(LinearRotationalPoint target)
{
  LengthUnit acceleration = target.linear_point.acceleration;
  LengthUnit     velocity = target.linear_point.velocity;

  double output = ffw_.output(acceleration.meters(), velocity.meters());

  return Matrix<double>::splat(output);
}

}
