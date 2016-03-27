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

  while (micros() - last_reading_time_ < RANGE_SENSOR_OFF_TIME) {
    // wait until this sensor has been off for long enough to turn it on again
  }

  off_reading = analogRead(pin_);
  digitalWrite(emitter_pin_, HIGH);

  delayMicroseconds(RANGE_SENSOR_ON_TIME);

  on_reading = analogRead(pin_);
  digitalWrite(emitter_pin_, LOW);
  last_raw_reading_ = on_reading - off_reading;
  last_reading_time_ = micros();

  if (sensed_distance < constants_.v0) {
    sensed_distance = (constants_.a1 * pow((on_reading - off_reading
            - constants_.b1), constants_.c1) + constants_.d1) + constants_.e;
  } else {
    sensed_distance = (constants_.a2 * pow((on_reading - off_reading
            - constants_.b2), constants_.c2) + constants_.d2) + constants_.e;
  }

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

int RangeSensor::getRawReading() {
    return last_raw_reading_;
}

bool RangeSensor::isWall() {

  getRange();
  return sawWall;

}
