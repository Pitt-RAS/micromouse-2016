#ifndef RANGESENSOR_H
#define RANGESENSOR_H

struct node{
    int range;
    float distance;
    node *next;
};

#define maxQueueLength 10

class RangeSensor {
private:
    node * rawRoot;
    node * averageRoot;
    int pin;
    //Last Update Time
public:
    RangeSensor(int tempPin);
    int getRange();
    int getLastRange() { return rawRoot->range; }
    int getRangeAtIndex(int index);
};

#endif

