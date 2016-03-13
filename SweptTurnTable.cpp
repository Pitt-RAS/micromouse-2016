#include "Arduino.h"

#include "conf.h"
#include "SweptTurnTable.h"
 
SweptTurnTable::SweptTurnTable(float for_speed, float a, float i_turn_rad, float a_acc, float m_a_vel, float m_w){
  forward_speed = for_speed;
  angle = a;
  ideal_turn_radius = i_turn_rad;
  angular_acceleration = a_acc;
  max_angular_velocity = m_a_vel;
  mouse_width = m_w;

  generateTimes();

  TableInfo generate_table[1000];
  generate_table[0] = {0,0};
  turn_table_[0] = 0;

  int i = 0;
  for(i = 1; i<TOTAL_TURN_TIME;i++){
    TableInfo cell_info;
    cell_info.velocity = getVelocity(i, generate_table[i-1].velocity); 
    cell_info.angle = getAngle(cell_info.velocity,generate_table[i-1].angle);
    generate_table[i] = cell_info;
    turn_table_[i] = cell_info.angle;
  }
}

float SweptTurnTable::getAngleAtIndex(int index)
{
  if (index <= total_actual_time_ / 2) {
    return turn_table_[index];
  }
  else {
    return angle - turn_table_[total_actual_time_ - index];
  }
}

float SweptTurnTable::getAngularAcceleration(int index)
{
  if (index < 0) {
    return 0;
  }
  else if (index < accel_time_) {
    return angular_acceleration;
  }
  else if (index < accel_time_ + const_time_) {
    return 0;
  }
  else if (index < total_actual_time_) {
    return - angular_acceleration;
  }

  return 0;
}

float SweptTurnTable::getTotalTime()
{
  return total_actual_time_;
}

float SweptTurnTable::getAngle(float velocity, float previous_angle)
{
  return previous_angle + velocity/TOTAL_TURN_TIME;
}
float SweptTurnTable::getVelocity(int time, float previous_velocity)
{
  if(time <= total_actual_time_){
    if(time <= accel_time_){
      return (float)angular_acceleration/TOTAL_TURN_TIME + previous_velocity;
    }
    else{
      if(time <= (accel_time_ + const_time_ / 2))
        return (float)max_angular_velocity;
      else
	return 0;
    }
  }
  return 0;
}
void SweptTurnTable::generateTimes()
{
  accel_time_ = (int)((max_angular_velocity/angular_acceleration)*1000+.5);
  const_time_ = (int)(((angle/max_angular_velocity)*1000+.5)-accel_time_);
  decell_time_ = accel_time_;
  total_actual_time_ = accel_time_+const_time_+decell_time_;
}
 
