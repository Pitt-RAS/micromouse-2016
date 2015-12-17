#include "conf.h"
#include "sensors_range.h"

float RangeSensor::read(int pin) {
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
    return offset + 699.02405*exp(-raw*0.0165939) + 50.35049;
}
