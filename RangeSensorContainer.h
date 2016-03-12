#ifndef RANGESENSORCONTAINER_H
#define RANGESENSORCONTAINER_H

#include "RangeSensor.h"

enum Direction {
	left = 0,
	front = 1,
	right = 2,
	back = 3
};


class RangeSensorContainer {
	private:
	
	public:
		RangeSensorContainer();
		RangeSensor leftSensor, rightSensor, frontSensor;

		void updateReadings();
		bool isWall(Direction wallToCheck); //Maybe directly return error for pid 
    	float errorFromCenter();

};

extern RangeSensorContainer RangeSensors;

#endif
