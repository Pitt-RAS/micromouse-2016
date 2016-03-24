#ifndef SWEPTTURNTABLE_H
#define SWEPTTURNTABLE_H

#include "conf.h"

struct TableInfo{
  float angle;
  float velocity;
};



class SweptTurnTable {
 private:
  int accel_time_;
  int const_time_;
  int total_actual_time_;
  float turn_table_[500];
  float forward_speed;
  float angle;
  float angular_acceleration;
  float max_angular_velocity;
  float getAngle(float velocity, float previous_angle);
  float getAngularVelocity(int time, float previous_velocity);
  void generateTimes();
 
 public:
  SweptTurnTable(float temp_forward_speed, float temp_angle, float temp_angular_acceleration, float temp_max_angular_velocity);
  float getAngleAtIndex(int index);
  float getAngularAcceleration(int index);
  float getTotalTime();
  
};

#endif

