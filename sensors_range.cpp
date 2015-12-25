#include "conf.h"
#include "sensors_range.h"

RangeSensor::RangeSensor() {
  UpdateRange();
}

void RangeSensor::UpdateRange () {
  UpdateRange(RANGE_DIAG_LEFT);
  UpdateRange(RANGE_DIAG_RIGHT);
  UpdateRange(RANGE_LEFT);
  UpdateRange(RANGE_RIGHT);
  UpdateRange(RANGE_FRONT);

}

void RangeSensor::UpdateRange (byte pin) {
  float raw_range = 208.5483 * exp(-0.003053 * analogRead(pin));
  
  switch (pin) {
    case RANGE_DIAG_LEFT:
      range_diag_left = RANGE_DIAG_LEFT_OFFSET + raw_range;
      break;
    case RANGE_DIAG_RIGHT:
      range_diag_right = RANGE_DIAG_RIGHT_OFFSET + raw_range;
      break;
    case RANGE_LEFT:
      range_left = RANGE_LEFT_OFFSET + raw_range;
      break;
    case RANGE_RIGHT:
      range_right = RANGE_RIGHT_OFFSET + raw_range;
      break;
    case RANGE_FRONT:
      range_front = RANGE_FRONT_OFFSET + raw_range;
      break;
  }
}

bool RangeSensor::IsWall (byte pin) {
  // in the future, potentially check if the addition of right and left is significantly more than the width of the cell, then say the smaller one has no wall and the larger one does. may not be a way to check if there is a wall on either side.
  switch (pin) {
    case RANGE_DIAG_LEFT:
      if (range_diag_left < DIAG_LEFT_WALL_THRESHOLD) {
        return 1;
      }
      break;
    case RANGE_DIAG_RIGHT:
      if (range_diag_right < DIAG_RIGHT_WALL_THRESHOLD) {
        return 1;
      }
      break;
    case RANGE_LEFT:
      if (range_left < LEFT_WALL_THRESHOLD) {
        return 1;
      }
      break;
    case RANGE_RIGHT:
      if (range_right < RIGHT_WALL_THRESHOLD) {
        return 1;
      }
      break;
    case RANGE_FRONT:
      if (range_front < FRONT_WALL_THRESHOLD) {
        return 1;
      }
      break;
    return 0;
  }
}

float RangeSensor::GetRange (byte pin) {
  switch (pin) {
    case RANGE_DIAG_LEFT:
      return range_diag_left;
      break;
    case RANGE_DIAG_RIGHT:
      return range_diag_right;
      break;
    case RANGE_LEFT:
      return range_left;
      break;
    case RANGE_RIGHT:
      return range_right;
      break;
    case RANGE_FRONT:
      return range_front;
      break;
  }
}




