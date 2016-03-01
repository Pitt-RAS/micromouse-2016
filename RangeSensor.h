#ifndef RANGESENSOR_H
#define RANGESENSOR_H

#include "conf.h"

class RangeSensor {
 private:
  int raw_queue_[RANGE_QUEUE_MAX_LENGTH];
  int raw_queue_length_ = 0;
  int raw_queue_index_ = 0;
  int raw_queue_sum_ = 0;
 
  int history_queue_[HISTORY_QUEUE_MAX_LENGTH];
  int history_queue_index_ = 0;

  bool sawWall = true;

  int low_threshold_, high_threshold_;
  int pin_;
  int emitter_pin_;

  struct TranslationConstants {
    float a;
    float b;
    float c;
    float d;
    int e;
  };

  TranslationConstants constants_;

 public:
  RangeSensor(int temp_pin, int lowT, int highT);
  void refreshRawQueue();
  void clearRawQueue();
  void clearHistory();
  void updateRange();
  int getRange();
  int getRange(int index);
  
  bool isWall();
};

#endif

