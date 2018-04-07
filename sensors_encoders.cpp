#include <Arduino.h>
#include <EncoderPittMicromouse.h>
#include "conf.h"

EncoderPittMicromouse left_front_encoder(ENCODER_LF1_PIN, ENCODER_LF2_PIN);
EncoderPittMicromouse right_front_encoder(ENCODER_RF1_PIN, ENCODER_RF2_PIN);
EncoderPittMicromouse left_back_encoder(ENCODER_LB1_PIN, ENCODER_LB2_PIN);
EncoderPittMicromouse right_back_encoder(ENCODER_RB1_PIN, ENCODER_RB2_PIN);

float enc_left_front_read() {
    return (left_front_encoder.read() * MM_PER_STEP);
}
float enc_left_back_read() {
    return (left_back_encoder.read() * MM_PER_STEP);
}
float enc_right_front_read() {
    return (right_front_encoder.read() * MM_PER_STEP);
}
float enc_right_back_read() {
    return (right_back_encoder.read() * MM_PER_STEP);
}
void enc_left_front_write(float new_distance) {
    left_front_encoder.write( (int32_t)rint(new_distance / MM_PER_STEP) );
}

void enc_left_back_write(float new_distance) {
    left_back_encoder.write( (int32_t)rint(new_distance / MM_PER_STEP) );
}

void enc_right_front_write(float new_distance) {
    right_front_encoder.write( (int32_t)rint(new_distance / MM_PER_STEP) );
}

void enc_right_back_write(float new_distance) {
    right_back_encoder.write( (int32_t)rint(new_distance / MM_PER_STEP) );
}

float enc_left_front_velocity() {
    return (1000 * left_front_encoder.stepRate() * MM_PER_STEP);
}
float enc_left_back_velocity() {
    return (1000 * left_back_encoder.stepRate() * MM_PER_STEP);
}

float enc_right_front_velocity() {
    return (1000 * right_front_encoder.stepRate() * MM_PER_STEP);
}
float enc_right_back_velocity() {
    return (1000 * right_back_encoder.stepRate() * MM_PER_STEP);
}

float enc_left_front_extrapolate() {
    return (left_front_encoder.extrapolate() * MM_PER_STEP);
}

float enc_left_back_extrapolate() {
    return (left_back_encoder.extrapolate() * MM_PER_STEP);
}
float enc_right_front_extrapolate() {
    return (right_front_encoder.extrapolate() * MM_PER_STEP);
}
float enc_right_back_extrapolate() {
    return (right_back_encoder.extrapolate() * MM_PER_STEP);
}

