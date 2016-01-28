#ifndef RANGESENSOR_H
#define RANGESENSOR_H

#include "conf.h"

class RangeSensor {
 private:
  int distance_queue_[RANGE_QUEUE_LENGTH];
  int distance_queue_index_ = 0;
  int distance_queue_sum_ = 0;
  int pin_;
 public:
  RangeSensor(int temp_pin);
  void clearOutdatedValues();
  void clearAllValues();
  int getRange();
};

#endif

