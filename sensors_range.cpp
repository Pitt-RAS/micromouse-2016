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
    //Serial2.println(raw);
    //return offset + 699.02405*exp(-raw*0.0165939) + 50.35049;

    // equation calculated using measured values on our setup and an excel generated fit line.  exponential curve.  Minimum accurate measurement is 25-30mm
    return (offset + 208.5483 * exp(-0.003053 * raw));

}
