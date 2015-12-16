#include <Arduino.h>

class RangeSensor {
  public:
    // returns the distance in mm
    static float read(int pin);
};