#include <Arduino.h>

float range_read(struct range_sensor sensor);

struct range_sensor {
    uint8_t pin_in;
    uint8_t pin_out;
    char type;
};
