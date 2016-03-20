#include "Arduino.h"

#include "conf.h"
#include "RangeSensor.h"

RangeSensor::RangeSensor(int temp_pin, int lowT, int highT) {
  pin_ = temp_pin;

  switch (pin_) {
    case RANGE1_PIN:
      emitter_pin_ = EMITTER1_PIN;
      constants_ = RANGE1_TRANSLATION;
      break;

    case RANGE2_PIN:
      emitter_pin_ = EMITTER2_PIN;
      constants_ = RANGE2_TRANSLATION;
      break;

    case RANGE3_PIN:
      emitter_pin_ = EMITTER3_PIN;
      constants_ = RANGE3_TRANSLATION;
      break;

    case RANGE4_PIN:
      emitter_pin_ = EMITTER4_PIN;
      constants_ = RANGE4_TRANSLATION;
      break;
  }

  low_threshold_ = lowT;
  high_threshold_ = highT;
}

void RangeSensor::updateRange() {
  float off_reading, on_reading;
  float sensed_distance;

  off_reading = analogRead(pin_);
  digitalWrite(emitter_pin_, HIGH);

  delayMicroseconds(45);

  on_reading = analogRead(pin_);
  digitalWrite(emitter_pin_, LOW);

  delayMicroseconds(45);

  sensed_distance = (constants_.a * pow((on_reading - off_reading
          + constants_.b), constants_.c) + constants_.d) + constants_.e;

  last_reading_ = sensed_distance;
}

int RangeSensor::getRange() {
  if(sawWall) {
	sawWall = last_reading_ < high_threshold_;
  }
  else {
	sawWall = last_reading_ < low_threshold_;
  }

  return last_reading_;
}

int RangeSensor::getRange(int index) {
	return last_reading_;
}

bool RangeSensor::isWall() {

  getRange();
  return sawWall;

}
