#ifndef MICROMOUSE_SENSORS_RANGE_H_
#define MICROMOUSE_SENSORS_RANGE_H_

#include <Arduino.h>

class RangeSensor {
  public:
    // returns the distance in mm
    static float read(int pin);
};

#endif
