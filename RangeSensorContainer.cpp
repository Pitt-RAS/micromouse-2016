#include "RangeSensorContainer.h"
#include "conf.h"

RangeSensorContainer RangeSensors;

RangeSensorContainer::RangeSensorContainer() 
	: leftSensor(RANGE_DIAG_LEFT_PIN, LEFT_LOW_THRESHOLD, LEFT_HIGH_THRESHOLD), rightSensor(RANGE_DIAG_RIGHT_PIN, RIGHT_LOW_THRESHOLD, RIGHT_HIGH_THRESHOLD), frontSensor(RANGE_FRONT_PIN, FRONT_LOW_THRESHOLD, FRONT_HIGH_THRESHOLD)
{
}

void RangeSensorContainer::updateReadings() {
	leftSensor.updateRange();
	rightSensor.updateRange();
	frontSensor.updateRange();
}

bool RangeSensorContainer::isWall(Direction wallToCheck) {
	
	switch (wallToCheck) {
	case left:
		return leftSensor.isWall();
		break;
	case front:
		return frontSensor.isWall();
		break;
	case right:
		return rightSensor.isWall();
		break;
	case back:
		return false;
		break;
	}

	return false;
}

void RangeSensorContainer::saveIsWall()
{
  saved_left_ = leftSensor.isWall();
  saved_right_ = rightSensor.isWall();
}

bool RangeSensorContainer::savedIsWall(Direction wallToCheck) {

	switch (wallToCheck) {
	case left:
		return saved_left_;
		break;
	case front:
		return frontSensor.isWall();
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
    
    leftReading = leftSensor.getRange();
    rightReading = rightSensor.getRange();

    /*NEW
     if (frontLeftSensor.getRange(1) < RANGE_DIAG_CUTOFF_FRONT_DISTANCE && frontRightSensor.getRange(1) < RANGE_DIAG_CUTOFF_FRONT_DISTANCE)
        return 0.0;
     */
    
    if (leftReading < RANGE_DIAG_LEFT_MIDDLE && rightReading < RANGE_DIAG_RIGHT_MIDDLE) {
        return 0.5 * (leftReading - rightReading - RANGE_DIAG_LEFT_MIDDLE + RANGE_DIAG_RIGHT_MIDDLE);
    }
    else if (leftReading < RANGE_DIAG_LEFT_MIDDLE) {
        return leftReading - RANGE_DIAG_LEFT_MIDDLE;
        
    }
    else if (rightReading < RANGE_DIAG_RIGHT_MIDDLE) {

        return RANGE_DIAG_RIGHT_MIDDLE - rightReading;
    }
    else
        return 0.0;

}
