#include "RangeSensor.h" 

RangeSensor::RangeSensor(byte tempPin){
	pin = tempPin;	
}

//Force to take new reading and adjust Queue
int RangeSensor::getRange(){
	
	node newRoot;
	newRoot.range = (int) 20760 / (analogRead(pin) - 11);
	newRoot.next = root;
	root = newRoot;

	//Iterate through the nodes until last node is found
	int i = 0;
	node queueItorator = root;	
	while(queueItorator.next.next != NULL) {
		queueItorator = queueItorator.next;
		i++;
	}

	//Remove last node if its depth is greater than queueLength
	if (i+1 >= queueLength) {
		free(queueItorator.next);
	}

	return getLastRange();
}

int RangeSensor::getRangeAtIndex(int index) {

	int i = 0;
	node queueItorator = root;	
	while(queueItorator.next.next != NULL && i < index) {
		queueItorator = queueItorator.next;
		i++;
	}

	return queueItorator.range;


}

