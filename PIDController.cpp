#include <Arduino.h>

#include "PIDController.h"

PIDController::PIDController(float tempKP, float tempKI, float tempKD, float temp_i_upper_bound,
                             float temp_i_lower_bound) {
  kp = tempKP;
  ki = tempKI;
  kd = tempKD;
  
  if (temp_i_lower_bound == 0) {
    temp_i_lower_bound = -temp_i_upper_bound;
  }

  i_lower_bound = temp_i_lower_bound;
  i_upper_bound = temp_i_upper_bound;
}

float PIDController::Calculate(float error) {
  i_term += ki * error * elapsed_time;
  i_term = constrain(i_term, i_lower_bound, i_upper_bound);

  float output = -kp * error - i_term + kd * (error - last_error) / elapsed_time;
  last_error = error;

  elapsed_time = 0;

  return output;
}
