#ifndef RANGESENSOR_H
#define RANGESENSOR_H

#include "conf.h"

class RangeSensor {
 private:
  int last_reading_ = 0;
 
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
  void updateRange();
  int getRange();
  int getRange(int index);
  
  bool isWall();
};

#endif

