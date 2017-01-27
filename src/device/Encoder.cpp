#include <cmath>
#include "Encoder.h"

Encoder gEncoderLF(4, 2);
Encoder gEncoderRF(8, 7);
Encoder gEncoderLB(5, 6);
Encoder gEncoderRB(9, 10);

Encoder::Encoder(uint8_t pin1, uint8_t pin2) : encoder_(pin1, pin2)
{}

double Encoder::count()
{
  return encoder_.extrapolate();
}

void Encoder::count(double value)
{
  encoder_.write(nearbyint(value));
}

double Encoder::countsPerSecond()
{
  return encoder_.stepRate();
}
