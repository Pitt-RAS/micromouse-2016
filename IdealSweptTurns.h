#ifndef IDEALSWEPTTURNS_H
#define IDEALSWEPTTURNS_H

#include "conf.h"

class IdealSweptTurns {
 private:
  float tangential_velocity;
  float turn_angle;
  float acceleration_duration;
  float const_velocity_duration;
  float turn_duration;
  float time_step;
  float frict_force;
  float inside_trigs;
  float mm_per_degree;

  float offset_table[500];
  
  float getAngleAtTime(float t);
  float getVelocityAtTime(float t);
  float getTurnOffset(float angle);
  
 public:
  IdealSweptTurns(float temp_tangential_velocity, float temp_turn_angle, float temp_time_step);
  float getOffsetAtMicros(unsigned long input_time);
};

#endif

