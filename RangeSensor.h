#ifndef RANGESENSOR_H
#define RANGESENSOR_H

#include "conf.h"

class RangeSensor {
 private:
  int raw_queue_[RANGE_QUEUE_MAX_LENGTH];
  int range_queue_length_ = 0;
  int raw_queue_index_ = 0;
  int raw_queue_sum_ = 0;
 
  int history_queue_[HISTORY_QUEUE_MAX_LENGTH];
  int history_queue_index_ = 0;
  
  int pin_;
 public:
  RangeSensor(int temp_pin);
  void refreshRawQueue();
  void clearRawQueue();
  int getRange();
};

#endif

