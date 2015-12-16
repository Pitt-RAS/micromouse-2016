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
    Serial2.print(raw);
    Serial2.print("\t");
    return offset + log((raw - 4.12374)/43.079)*-0.851567;
}
