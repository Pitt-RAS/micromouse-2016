#include "RangeSensor.h"
#include "Arduino.h"


RangeSensor::RangeSensor(int tempPin){
    pin = tempPin;
}

//Force to take new reading and adjust Queue
int RangeSensor::getRange(){
    
    node *newRoot = new node;
    newRoot->range = (int) 20760 / (analogRead(pin) - 11);
    newRoot->next = rawRoot;
    rawRoot = newRoot;
    
    node *queueItorator = rawRoot;
    for(int queueIndex = 1; queueItorator->next && queueIndex < maxQueueLength+1; queueIndex++) {
        if (queueIndex == maxQueueLength) {
            queueItorator->next = NULL;
        }
        else {
            queueItorator = queueItorator->next;
        }
    }
    
    return getLastRange();
}

int RangeSensor::getRangeAtIndex(int index) {
    
    node *queueItorator = rawRoot;
    for(int queueIndex = 1; queueItorator->next && queueIndex < maxQueueLength+1; queueIndex++) {
        if (queueIndex == index) {
            break;
        }
        queueItorator = queueItorator->next;
    }
    
    return queueItorator->range;
    
}


