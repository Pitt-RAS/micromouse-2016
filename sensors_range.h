#include <Arduino.h>

#ifndef SENSORS_RANGE_H_
#define SENSORS_RANGE_H_

// input current speed and desired force
class RangeSensor {
  private:
    float range_diag_left, range_diag_right, range_left, range_right, range_front;
  public:
    RangeSensor ();
    void UpdateRange ();
    void UpdateRange (byte pin);
    bool IsWall (byte pin);
    float GetRange (byte pin);
};

#endif
