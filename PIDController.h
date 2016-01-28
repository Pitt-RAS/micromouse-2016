#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
  private:
    float i_term = 0;
    elapsedMicros elapsed_time;
    float last_error = 0;
    float kp, ki, kd;
    float i_lower_bound, i_upper_bound;
  public:
    PIDController(float tempKP, float tempKI, float tempKD, float temp_i_upper_bound = 2,
                  float temp_i_lower_bound = 0);
    float Calculate(float error);
};

#endif
