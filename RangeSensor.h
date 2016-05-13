#ifndef RANGESENSOR_H
#define RANGESENSOR_H

#include "conf.h"

class RangeSensor {
 private:
  int last_reading_ = 0;
  int last_raw_reading_ = 0;
  uint32_t last_reading_time_ = 0;
 
  bool sawWall = true;

  int low_threshold_, high_threshold_;
  int pin_;
  int emitter_pin_;

  struct TranslationConstants {
    float a1, b1, c1, d1;
    float a2, b2, c2, d2;
    float v0;
    float e;
  };

  TranslationConstants constants_;

 public:
  RangeSensor(int temp_pin, int lowT, int highT);
  void updateRange();
  int getRange();
  int getRange(int index);

  // returns the raw result of the last reading
  int getRawReading();
  
  bool isWall();
};

#endif

