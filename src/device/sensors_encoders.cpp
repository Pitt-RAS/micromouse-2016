#include <cstdint>
#include <cmath>
#include "../motion/units.h"
#include "Encoder.h"
#include "sensors_encoders.h"

static const float kMillimetersPerCount = 0.653868;

static void write(Encoder &encoder, float value);
static float velocity(Encoder &encoder);
static float extrapolate(Encoder &encoder);

void  enc_left_front_write(float value) { write(gEncoderLF, value); }
void   enc_left_back_write(float value) { write(gEncoderLB, value); }
void enc_right_front_write(float value) { write(gEncoderRF, value); }
void  enc_right_back_write(float value) { write(gEncoderRB, value); }

float  enc_left_front_velocity() { return velocity(gEncoderLF); }
float   enc_left_back_velocity() { return velocity(gEncoderLB); }
float enc_right_front_velocity() { return velocity(gEncoderRF); }
float  enc_right_back_velocity() { return velocity(gEncoderRB); }

float  enc_left_front_extrapolate() { return extrapolate(gEncoderLF); }
float   enc_left_back_extrapolate() { return extrapolate(gEncoderLB); }
float enc_right_front_extrapolate() { return extrapolate(gEncoderRF); }
float  enc_right_back_extrapolate() { return extrapolate(gEncoderRB); }

static void write(Encoder &encoder, float value)
{
  encoder.displacement(Motion::LengthUnit::fromMeters(value / 1000.0));
}

static float velocity(Encoder &encoder)
{
  return 1000.0 * encoder.velocity().meters();
}

static float extrapolate(Encoder &encoder)
{
  return 1000.0 * encoder.displacement().meters();
}
