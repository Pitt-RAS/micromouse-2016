#include "RangeSensorContainer.h"
#include "conf.h"

RangeSensorContainer RangeSensors;

RangeSensorContainer::RangeSensorContainer() 
	: diagLeftSensor(RANGE_DIAG_LEFT_PIN, DIAG_LEFT_LOW_THRESHOLD, DIAG_LEFT_HIGH_THRESHOLD),
    diagRightSensor(RANGE_DIAG_RIGHT_PIN, DIAG_RIGHT_LOW_THRESHOLD, DIAG_RIGHT_HIGH_THRESHOLD),
    frontLeftSensor(RANGE_FRONT_LEFT_PIN, FRONT_LEFT_LOW_THRESHOLD, FRONT_LEFT_HIGH_THRESHOLD),
    frontRightSensor(RANGE_FRONT_RIGHT_PIN, FRONT_RIGHT_LOW_THRESHOLD, FRONT_RIGHT_HIGH_THRESHOLD)
{
}

void RangeSensorContainer::updateReadings() {
	diagLeftSensor.updateRange();
	diagRightSensor.updateRange();
	frontLeftSensor.updateRange();
	frontRightSensor.updateRange();
}

bool RangeSensorContainer::isWall(Direction wallToCheck) {
  switch (wallToCheck) {
    case left:
      return diagLeftSensor.isWall();
      break;
    case front:
      return frontLeftSensor.getRange() + frontRightSensor.getRange() < FRONT_LEFT_LOW_THRESHOLD * 2;
      break;
    case right:
      return diagRightSensor.isWall();
      break;
    case back:
      return false;
      break;
  }
  
  return false;
}

void RangeSensorContainer::saveIsWall()
{
  saved_left_ = diagLeftSensor.isWall();
  saved_right_ = diagRightSensor.isWall();
}

bool RangeSensorContainer::savedIsWall(Direction wallToCheck) {

	switch (wallToCheck) {
	case left:
		return saved_left_;
		break;
	case front:
		return frontLeftSensor.isWall();
		break;
	case right:
		return saved_right_;
		break;
	case back:
		return false;
		break;
	}

	return false;
}

float RangeSensorContainer::errorFromCenter() {
  float leftReading, rightReading, frontReading;

  leftReading = diagLeftSensor.getRange(1);
  rightReading = diagRightSensor.getRange(1);
  frontReading = min(frontLeftSensor.getRange(1), frontRightSensor.getRange(1));

  if (frontReading < RANGE_DIAG_CUTOFF_FRONT_DISTANCE) {
    return 0.0;
  } else if (leftReading < RANGE_DIAG_LEFT_MIDDLE && rightReading < RANGE_DIAG_RIGHT_MIDDLE) {
    return 0.5 * (leftReading - rightReading - RANGE_DIAG_LEFT_MIDDLE + RANGE_DIAG_RIGHT_MIDDLE);
  } else if (leftReading < RANGE_DIAG_LEFT_MIDDLE) {
    return leftReading - RANGE_DIAG_LEFT_MIDDLE;
  } else if (rightReading < RANGE_DIAG_RIGHT_MIDDLE) {
    return RANGE_DIAG_RIGHT_MIDDLE - rightReading;
  } else {
    return 0.0;
  }
}
