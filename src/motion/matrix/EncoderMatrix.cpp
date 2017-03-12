#include "../units.h"
#include "EncoderMatrix.h"

namespace Motion {

EncoderMatrix gEncoder(
  gEncoderLF, gEncoderRF,
  gEncoderLB, gEncoderRB
);

EncoderMatrix::EncoderMatrix(Encoder &a, Encoder &b, Encoder &c, Encoder &d) :
  Matrix<Encoder&>(a, b, c, d)
{}

void EncoderMatrix::zero()
{
  forEach(
    [] (Encoder &encoder) -> void {
      encoder.displacement(LengthUnit::zero());
    }
  );
}

Matrix<LengthUnit> EncoderMatrix::linearDisplacement()
{
  Matrix<double> raw_abstract = map<double>(
    [] (Encoder &encoder) -> double {
      return encoder.displacement().abstract();
    }
  );

  Matrix<double> straightened_abstract = raw_abstract.straighten();

  return straightened_abstract.map<LengthUnit>(
    [] (double &abstract) -> LengthUnit {
      return LengthUnit::fromAbstract(abstract);
    }
  );
}

LengthUnit EncoderMatrix::averageDisplacement()
{
  Matrix<double> abstract = linearDisplacement().map<double>(
    [] (LengthUnit &length) -> double {
      return length.abstract();
    }
  );

  double average_abstract = abstract.average();

  return LengthUnit::fromAbstract(average_abstract);
}

void EncoderMatrix::zeroLinearDisplacement(LengthUnit new_zero)
{
  Matrix<LengthUnit> value = Matrix<LengthUnit>::splat(new_zero);

  forEachWith<LengthUnit>(
    value,
    [] (Encoder &encoder, LengthUnit &value) -> void {
      encoder.zeroDisplacement(value);
    }
  );
}

}
