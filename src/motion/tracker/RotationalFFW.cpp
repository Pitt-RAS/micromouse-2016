#include "RotationalFFW.h"

namespace Motion {

RotationalFFW::RotationalFFW(FFWParameters parameters) : ffw_(parameters)
{}

Matrix<double> RotationalFFW::output(LinearRotationalPoint target)
{
  AngleUnit acceleration = target.rotational_point.acceleration;
  AngleUnit     velocity = target.rotational_point.velocity;

  double output = ffw_.output(acceleration.radians(), velocity.radians());

  return Matrix<double>::splat(output).oppose();
}

}
