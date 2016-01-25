#include "RangeSensor.h"
#include "Arduino.h"


RangeSensor::RangeSensor(int tempPin){
    pin = tempPin;
}

//Force to take new reading and adjust Queue
int RangeSensor::getRange(){
    
    node *newRawRoot = new node;
    newRawRoot->range = (int) 20760 / (analogRead(pin) - 11);
    newRawRoot->next = rawRoot;
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
    averageRoot = newAvgRoot;
   
    return getLastRange();
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


