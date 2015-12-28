#ifndef MICROMOUSE_SENSORS_ENCODERS_H_
#define MICROMOUSE_SENSORS_ENCODERS_H_

#include <Arduino.h>

float enc_left_read();
float enc_right_read();
void enc_left_write(float NEW_DISTANCE);
void enc_right_write(float NEW_DISTANCE);
float enc_left_velocity();
float enc_right_velocity();
float enc_left_extrapolate();
float enc_right_extrapolate();

#endif
