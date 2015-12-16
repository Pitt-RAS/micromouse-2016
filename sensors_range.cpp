#include "sensors_range.h"

RangeSensor::read(int pin) {
    float offset;
    switch (pin) {
        case RANGE_FRONT_LEFT:
            offset = RANGE_FRONT_LEFT_OFFSET;
            break;
        case RANGE_FRONT_RIGHT:
            offset = RANGE_FRONT_RIGHT_OFFSET;
            break;
        case RANGE_LEFT:
            offset = RANGE_LEFT_OFFSET;
            break;
        case RANGE_RIGHT:
            offset = RANGE_RIGHT_OFFSET;
            break;
        case RANGE_FRONT:
            offset = RANGE_FRONT_OFFSET;
            break;
    }

    int raw = analogRead(pin);
    return offset + 43.079*exp(-raw*0.78287) + 4.12374;
}