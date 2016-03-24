#ifndef MICROMOUSE_SENSORS_ENCODERS_H_
#define MICROMOUSE_SENSORS_ENCODERS_H_

#include <Arduino.h>

float enc_left_front_read();
float enc_left_back_read();
float enc_right_front_read();
float enc_right_back_read();
void enc_left_front_write(float);
void enc_left_back_write(float);
void enc_right_front_write(float);
void enc_right_back_write(float);
float enc_left_front_velocity();
float enc_left_back_velocity();
float enc_right_front_velocity();
float enc_right_back_velocity();
float enc_left_front_extrapolate();
float enc_left_back_extrapolate();
float enc_right_front_extrapolate();
float enc_right_back_extrapolate();

#endif
