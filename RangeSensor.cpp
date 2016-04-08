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

    case RANGE5_PIN:
      emitter_pin_ = EMITTER5_PIN;
      constants_ = RANGE5_TRANSLATION;
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
  //NEW// delayMicroseconds(RANGE_SENSOR_ON_TIME);

  on_reading = analogRead(pin_);
  digitalWrite(emitter_pin_, LOW);

  delayMicroseconds(45); // Depricated in NEW

  if (on_reading - off_reading + constants_.b <= 0) {
    last_reading_ = 10000;
  }
  else {
    sensed_distance = (constants_.a * pow((on_reading - off_reading
            + constants_.b), constants_.c) + constants_.d) + constants_.e;

    last_reading_ = sensed_distance;
  }
    
  /* NEW
    if (last_raw_reading_ < constants_.v0) {
        if (last_raw_reading_ - constants_.b1 < 0) {
            sensed_distance = INFINITY;
        } else {
            sensed_distance = (constants_.a1 * pow(last_raw_reading_
                                                   - constants_.b1, constants_.c1) + constants_.d1) + constants_.e;
        }
    } else {
        if (last_raw_reading_ - constants_.b2 < 0) {
            sensed_distance = INFINITY;
        } else {
            sensed_distance = (constants_.a2 * pow(last_raw_reading_
                                                   - constants_.b2, constants_.c2) + constants_.d2) + constants_.e;
        }
    }
   */
    
}


int RangeSensor::getRange() {

  return last_reading_;
}

int RangeSensor::getRange(int index) {
	return last_raw_reading_;
}

bool RangeSensor::isWall() {

  getRange();
  return sawWall;

}
