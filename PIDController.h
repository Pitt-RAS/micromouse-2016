#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
  private:
    float i_term = 0;
    elapsedMicros elapsed_time;
    float last_value = NAN;
    float kp, ki, kd;
    float i_lower_bound, i_upper_bound;
  public:
    PIDController(float tempKP, float tempKI, float tempKD, float temp_i_upper_bound = 10000,
                  float temp_i_lower_bound = 0);
    float Calculate(float current_value, float target_value);
};

#endif
