#include <Arduino.h>
#include <Encoder.h>
#include "conf.h"

Encoder left_encoder(ENCODER_A1, ENCODER_A2);
Encoder right_encoder(ENCODER_B1, ENCODER_B2);

int32_t enc_left_read() {
    return left_encoder.read();
}

int32_t enc_right_read() {
    return right_encoder.read();
}
