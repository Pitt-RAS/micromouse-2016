#include "Arduino.h"

#include "conf.h"
#include "RangeSensor.h"

RangeSensor::RangeSensor(int temp_pin) {
  pin_ = temp_pin;
  memset(distance_queue_, 0, sizeof(distance_queue_));
}

void RangeSensor::clearOutdatedValues() {
  int current_average = distance_queue_sum_ / RANGE_QUEUE_LENGTH;
  for (int i = 0; i < RANGE_QUEUE_NUM_TO_CLEAR; i++) {
    distance_queue_sum_ -= distance_queue_[distance_queue_index_];
    distance_queue_[distance_queue_index_] = current_average;
    distance_queue_sum_ += distance_queue_[distance_queue_index_];
    distance_queue_index_ = (distance_queue_index_ + 1) % RANGE_QUEUE_LENGTH;
  }
}

void RangeSensor::clearAllValues() {
  distance_queue_index_ = 0;
  distance_queue_sum_ = 0;
  memset(distance_queue_, 0, sizeof(distance_queue_));
}

//Force to take new reading and adjust Queue
int RangeSensor::getRange() {
  distance_queue_sum_ -= distance_queue_[distance_queue_index_];
  distance_queue_[distance_queue_index_] = (int) 20760 / (analogRead(pin_) - 11);
  distance_queue_sum_ += distance_queue_[distance_queue_index_];
  distance_queue_index_ = (distance_queue_index_ + 1) % RANGE_QUEUE_LENGTH;
  
  return distance_queue_sum_ / RANGE_QUEUE_LENGTH;
}

