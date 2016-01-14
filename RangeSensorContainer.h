#ifdef RANGESENSORCONTAINER_H
#define RANGESENSORCONTAINER_H

#include "RangeSensor.h"

enum {
	left = 0,
	front = 1,
	right = 2,
	back = 3
} Direction;


class RangeSensorContainer {
	private:
	
	public:
		RangeSensor leftSensor, rightSensor;

		void updateReadings();
		bool isWall(Direction wallToCheck); //Maybe directly return error for pid 

};

#endif
