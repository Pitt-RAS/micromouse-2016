#ifndef MICROMOUSE_UTILITY_H_
#define MICROMOUSE_UTILITY_H_

#include <Arduino.h>

bool knowsBestPath(size_t target_x, size_t target_y);
void streamRanges();
void streamRawRanges();

#endif
