#ifndef SWEPTTURNTABLE_H
#define SWEPTTURNTABLE_H
#define TOTAL_TURN_TIME 1000.0
#include "conf.h"

struct TableInfo{
  float angle;
  float velocity;
};



class SweptTurnTable {
 private:
  int accel_time_;
  int const_time_;
  int decell_time_;
  int total_actual_time_;
  float turn_table_[1000];
  float forward_speed;
  float angle;
  float ideal_turn_radius;
  float angular_acceleration;
  float max_angular_velocity;
  float mouse_width;
  float getAngle(float velocity, float previous_angle);
  float getVelocity(int time, float previous_velocity);
  void generateTimes();
 
 public:
  SweptTurnTable(float for_speed, float a, float i_turn_rad, float a_acc, float m_a_vel, float m_w);
  float getAngleAtIndex(int index);
  float getAngularAcceleration(int index);
  float getTotalTime();
  
};

#endif

