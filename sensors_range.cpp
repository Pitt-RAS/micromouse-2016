#include "conf.h"
#include "sensors_range.h"

float RangeSensor::read(int pin) {
    float Offset;
    switch (pin) {
        case RANGE_FRONT_LEFT:
            Offset = RANGE_FRONT_LEFT_OFFSET;
            break;
        case RANGE_FRONT_RIGHT:
            Offset = RANGE_FRONT_RIGHT_OFFSET;
            break;
        case RANGE_LEFT:
            Offset = RANGE_LEFT_OFFSET;
            break;
        case RANGE_RIGHT:
            Offset = RANGE_RIGHT_OFFSET;
            break;
        case RANGE_FRONT:
            Offset = RANGE_FRONT_OFFSET;
            break;
    }

    int raw = analogRead(pin);

    Serial2.print(raw);
    Serial2.print("\t");
    return Offset + log((raw - 4.12374)/43.079)*-0.851567;

}
