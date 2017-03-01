#ifndef MICROMOUSE_ENCODER_H
#define MICROMOUSE_ENCODER_H

#include <cstdint>
#include <EncoderPittMicromouse.h>
#include "../motion/Matrix.h"
#include "../motion/units.h"

class Encoder
{
  public:
    Encoder(uint8_t pin1, uint8_t pin2);

    Motion::LengthUnit displacement();
    void displacement(Motion::LengthUnit value);

    Motion::LengthUnit velocity();

  private:
    EncoderPittMicromouse encoder_;
};

extern Encoder gEncoderLF, gEncoderRF, gEncoderLB, gEncoderRB;
extern Motion::Matrix<Encoder&> gEncoder;

#endif
