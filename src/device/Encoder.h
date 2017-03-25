#ifndef MICROMOUSE_ENCODER_H
#define MICROMOUSE_ENCODER_H

#include <cstdint>
#include <EncoderPittMicromouse.h>

class Encoder
{
  public:
    Encoder(uint8_t pin1, uint8_t pin2);

    // extrapolated
    double count();
    void count(double value);

    double countsPerSecond();

    EncoderPittMicromouse &innerObject();

  private:
    EncoderPittMicromouse encoder_;
};

extern Encoder gEncoderLF, gEncoderRF, gEncoderLB, gEncoderRB;

#endif
