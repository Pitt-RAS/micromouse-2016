#include "conf.h"
#include "sensors_range.h"

RangeSensor RangeSensor;

RangeSensor::RangeSensor() {
  UpdateRange();
}

void RangeSensor::UpdateRange () {
  UpdateRange(RANGE_DIAG_LEFT_PIN);
  UpdateRange(RANGE_DIAG_RIGHT_PIN);
  UpdateRange(RANGE_LEFT_PIN);
  UpdateRange(RANGE_RIGHT_PIN);
  UpdateRange(RANGE_FRONT_PIN);

}

void RangeSensor::UpdateRange (byte pin) {
  //float raw_range = 208.5483 * exp(-0.003053 * analogRead(pin));
  float raw_range = (float) 20760 / (analogRead(pin) - 11);
  
  switch (pin) {
    case RANGE_DIAG_LEFT_PIN:
      range_diag_left = RANGE_DIAG_LEFT_OFFSET + raw_range;
      break;
    case RANGE_DIAG_RIGHT_PIN:
      range_diag_right = RANGE_DIAG_RIGHT_OFFSET + raw_range;
      break;
    case RANGE_LEFT_PIN:
      range_left = RANGE_LEFT_OFFSET + raw_range;
      break;
    case RANGE_RIGHT_PIN:
      range_right = RANGE_RIGHT_OFFSET + raw_range;
      break;
    case RANGE_FRONT_PIN:
      range_front = RANGE_FRONT_OFFSET + raw_range;
      break;
  }
}

bool RangeSensor::IsWall (byte pin) {
  // in the future, potentially check if the addition of right and left is significantly more than the width of the cell, then say the smaller one has no wall and the larger one does. may not be a way to check if there is a wall on either side.
  switch (pin) {
    case RANGE_DIAG_LEFT_PIN:
      if (range_diag_left < RANGE_DIAG_LEFT_WALL_THRESHOLD) {
        return 1;
      }
      break;
    case RANGE_DIAG_RIGHT_PIN:
      if (range_diag_right < RANGE_DIAG_RIGHT_WALL_THRESHOLD) {
        return 1;
      }
      break;
    case RANGE_LEFT_PIN:
      if (range_left < RANGE_LEFT_WALL_THRESHOLD) {
        return 1;
      }
      break;
    case RANGE_RIGHT_PIN:
      if (range_right < RANGE_RIGHT_WALL_THRESHOLD) {
        return 1;
      }
      break;
    case RANGE_FRONT_PIN:
      if (range_front < RANGE_FRONT_WALL_THRESHOLD) {
        return 1;
      }
      break;
    return 0;
  }
}

float RangeSensor::GetRange (byte pin) {
  switch (pin) {
    case RANGE_DIAG_LEFT_PIN:
      return range_diag_left;
      break;
    case RANGE_DIAG_RIGHT_PIN:
      return range_diag_right;
      break;
    case RANGE_LEFT_PIN:
      return range_left;
      break;
    case RANGE_RIGHT_PIN:
      return range_right;
      break;
    case RANGE_FRONT_PIN:
      return range_front;
      break;
  }
}




