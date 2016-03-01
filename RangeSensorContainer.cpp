#include "RangeSensorContainer.h"
#include "conf.h"

RangeSensorContainer RangeSensors;

RangeSensorContainer::RangeSensorContainer() 
	: leftSensor(RANGE_DIAG_LEFT_PIN, LEFT_LOW_THRESHOLD, LEFT_HIGH_THRESHOLD), rightSensor(RANGE_DIAG_RIGHT_PIN, RIGHT_LOW_THRESHOLD, RIGHT_HIGH_THRESHOLD)
{
}

void RangeSensorContainer::updateReadings() {
	leftSensor.updateRange();
	rightSensor.updateRange();
}

bool RangeSensorContainer::isWall(Direction wallToCheck) {
	
	switch (wallToCheck) {
		case left:
				return leftSensor.isWall();
			break;
		case front:
			break;
		case right:
				return rightSensor.isWall();
			break;
    case back:
      break;
	}

	return false;
}

float RangeSensorContainer::errorFromCenter() {
  float errorCenter;
	if (isWall(left) && isWall(right)) {
		errorCenter = .5 * (leftSensor.getRange(1) - rightSensor.getRange(1));
	}
	else if (isWall(left)) {
		errorCenter = (leftSensor.getRange(1) - RANGE_DIAG_LEFT_MIDDLE);
	}
	else if (isWall(right)) {
		errorCenter = (RANGE_DIAG_RIGHT_MIDDLE - rightSensor.getRange(1));
	}
	else {
		errorCenter = 0;
	}

  return errorCenter;
}
