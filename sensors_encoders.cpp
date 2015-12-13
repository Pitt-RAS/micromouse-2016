#include <Arduino.h>
#include <EncoderMod.h>
#include "conf.h"

Encoder left_encoder(ENCODER_A1, ENCODER_A2);
Encoder right_encoder(ENCODER_B1, ENCODER_B2);

float enc_left_read() {
    return (left_encoder.read() * MM_PER_STEP);
}

float enc_right_read() {
    return (right_encoder.read() * MM_PER_STEP);
}

void enc_left_write(float new_distance) {
    left_encoder.write( (int32_t)rint(new_distance / MM_PER_STEP) );
}

void enc_right_write(float new_distance) {
    right_encoder.write( (int32_t)rint(new_distance / MM_PER_STEP) );
}

float enc_left_velocity() {
    return (1000 * left_encoder.stepRate() * MM_PER_STEP);
}

float enc_right_velocity() {
    return (1000 * right_encoder.stepRate() * MM_PER_STEP);
}

float enc_left_extrapolate() {
    return (left_encoder.extrapolate() * MM_PER_STEP);
}
float enc_right_extrapolate() {
    return (right_encoder.extrapolate() * MM_PER_STEP);
}

