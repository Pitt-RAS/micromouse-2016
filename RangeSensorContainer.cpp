#include "RangeSensorContainer.h"
	
	RangeSensorContainer::RangeSensorContainer() 
	: leftSensor(RANGE_DIAG_LEFT_PIN), rightSensor(RANGE_DIAG_RIGHT_RIGHT_PIN)
{


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
			if (rightSesnor.getRange() < RANGE_DIAG_RIGHT_WALL_THRESHOLD) {
				return true;
			}
			break;
	}

	return false;
}
