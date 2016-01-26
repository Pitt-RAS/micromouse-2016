#include "RangeSensor.h"
#include "Arduino.h"


RangeSensor::RangeSensor(int tempPin){
    pin = tempPin;
}

//Force to take new reading and adjust Queue
int RangeSensor::getRange(int elapsedSteps){
    
    node *newRawRoot = new node;
    newRawRoot->range = (int) 20760 / (analogRead(pin) - 11);
    newRawRoot->next = rawRoot;
	newRawRoot->distance = elapsedSteps
	if(rawRoot) 
		newRawRoot->distance += rawRoot->distance
    rawRoot = newRawRoot;
    
    node *queueItorator = rawRoot;
	int sum = rawRoot->range;
	int queueIndex = 1;
    for(int queueIndex = 1; queueItorator->next && queueIndex < maxQueueLength+1; queueIndex++) {
        if (queueIndex == maxQueueLength) {
            queueItorator->next = NULL;
        }
        else {
            queueItorator = queueItorator->next;
			sum += queueItorator->range;
        }
    }
 
	node *newAvgRoot = new node;
    newAvgRoot->range = sum / queueIndex;
    newAvgRoot->next = averageRoot;
	if(averageRoot)
		newAvgRoot->distance = averageRoot->distance
    averageRoot = newAvgRoot;
   
    return getRange();
}

int RangeSensor::getRangeAtIndex(int index) {
    
    node *queueItorator = averageRoot;
    for(int queueIndex = 1; queueItorator->next && queueIndex < maxQueueLength+1; queueIndex++) {
        if (queueIndex == index) {
    		return queueItorator->range;
        }
        queueItorator = queueItorator->next;
    }
	
	return NULL;

}


