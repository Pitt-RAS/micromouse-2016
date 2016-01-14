#ifndef RANGESENSOR_H
#define RANGESENSOR_H

struct node{
	int range;
	node *next;
};

#define queueLength 10

class RangeSensor {
	private:
		node root;
		byte pin;
		//Last Update Time
	public:
		RangeSensor(byte tempPin);
		int getRange();
		int getLastRange() { return root.range; }
		int getRangeAtIndex(int index);
};

#endif
