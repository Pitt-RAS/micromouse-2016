#include "Arduino.h"

#include "conf.h"
#include "RangeSensor.h"

RangeSensor::RangeSensor(int temp_pin, int lowT, int highT) {
  pin_ = temp_pin;
  low_threshold_ = lowT;
  high_threshold_ = highT;
  memset(raw_queue_, 0, sizeof(raw_queue_));
  memset(history_queue_, 0, sizeof(history_queue_));
}

void RangeSensor::refreshRawQueue() {
	
  for (int i = 0; i < RANGE_QUEUE_NUM_TO_CLEAR; i++) {
    raw_queue_sum_ -= raw_queue_[(raw_queue_index_+ i) % RANGE_QUEUE_MAX_LENGTH];
    raw_queue_[raw_queue_index_] = 0;
	raw_queue_length_--;
  }
}

void RangeSensor::clearRawQueue() {
  raw_queue_index_ = 0;
  raw_queue_sum_ = 0;
  raw_queue_length_ = 0;
  memset(raw_queue_, 0, sizeof(raw_queue_));
}

void RangeSensor::clearHistory() {
	history_queue_index_ = 0;
	memset(history_queue_, 0, sizeof(history_queue_));
}

//Update Raw Range Queue so ranges are accurate later when needed by higher level
void RangeSensor::updateRange() {
  raw_queue_sum_ -= raw_queue_[raw_queue_index_];
  raw_queue_[raw_queue_index_] = (int) 20760 / (analogRead(pin_) - 11);
  raw_queue_sum_ += raw_queue_[raw_queue_index_];
  raw_queue_index_ = (raw_queue_index_ + 1) % RANGE_QUEUE_MAX_LENGTH;

  if(raw_queue_length_ != RANGE_QUEUE_MAX_LENGTH) {
	  raw_queue_length_++;
  }
}

//Force to take new reading and adjust Queue
int RangeSensor::getRange() {

  int raw_queue_average_ = raw_queue_sum_ / raw_queue_length_;

  history_queue_[history_queue_index_] = raw_queue_average_;
  history_queue_index_ = (history_queue_index_ + 1) % HISTORY_QUEUE_MAX_LENGTH ;

  if(sawWall) {
	sawWall = raw_queue_average_ < high_threshold_;
  }
  else {
	sawWall = raw_queue_average_ < low_threshold_;
  }
  
  return raw_queue_average_;
}

int RangeSensor::getRange(int index) {
	if(!index) {
		return getRange();
	}
		
	if(history_queue_index_-index >= 0) {
		return history_queue_[history_queue_index_-index];
	}
	else {	
		return history_queue_[HISTORY_QUEUE_MAX_LENGTH+history_queue_index_-index];
	}

}

bool RangeSensor::isWall() {

  getRange();
  return sawWall;

}
