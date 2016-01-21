#include "RangeSensorContainer.h"
#include "conf.h"

	RangeSensorContainer::RangeSensorContainer() 
	: leftSensor(RANGE_DIAG_LEFT_PIN), rightSensor(RANGE_DIAG_RIGHT_PIN)
{
  Serial.printf("RangeContainer Made DEBUG");

}

void RangeSensorContainer::updateReadings() {
	leftSensor.getRange();
	rightSensor.getRange();
}

//Schmidt Trigger, Time Adjust, Varrying Threshold, getLastRange, Reset 
bool RangeSensorContainer::isWall(Direction wallToCheck) {
	switch (wallToCheck) {
		case left:
			if (leftSensor.getRange() < RANGE_DIAG_LEFT_WALL_THRESHOLD) {
				return true;
			}
			break;
		case front:
			break;
		case right:
			if (rightSensor.getRange() < RANGE_DIAG_RIGHT_WALL_THRESHOLD) {
				return true;
			}
			break;
    case back:
      break;
	}

	return false;
}

float RangeSensorContainer::errorFromTarget(float target) {
  float errorCenter;
	updateReadings();
	if (isWall(left) && isWall(right)) {
		errorCenter = .5 * leftSensor.getRange() - rightSensor.getRange();
	}
	else if (isWall(left)) {
		errorCenter = (leftSensor.getRange() - RANGE_DIAG_LEFT_MIDDLE);
	}
	else if (isWall(right)) {
		errorCenter = (RANGE_DIAG_RIGHT_MIDDLE - rightSensor.getRange());
	}
	else {
		errorCenter = 0;
	}

  return errorCenter;
}
