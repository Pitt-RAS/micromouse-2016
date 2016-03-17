#include "Arduino.h"

#include "conf.h"
#include "SweptTurnTable.h"
 
SweptTurnTable::SweptTurnTable(float temp_forward_speed, float temp_angle, float temp_angular_acceleration, float temp_max_angular_velocity){
  forward_speed = temp_forward_speed;
  angle = temp_angle;
  angular_acceleration = temp_angular_acceleration;
  max_angular_velocity = temp_max_angular_velocity;

  generateTimes();

  TableInfo generate_table[500];
  generate_table[0] = {0,0};
  turn_table_[0] = 0;

  int i = 0;
  for(i = 1; i < (int)((total_actual_time_ / 2) + .5);i++){
    TableInfo cell_info;
    cell_info.velocity = getAngularVelocity(i, generate_table[i-1].velocity); 
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
  else if (index <= total_actual_time_) {
    return angle - turn_table_[total_actual_time_ - index];
  }
  else {
    return angle;
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
  return previous_angle + velocity/total_actual_time_;
}

float SweptTurnTable::getAngularVelocity(int time, float previous_velocity)
{
  if(time <= total_actual_time_){
    if(time <= accel_time_){
      return (float)angular_acceleration/total_actual_time_ + previous_velocity;
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

// generates the time segments in which the mouse is accelerating/decelerating or moving at a constant speed.  
void SweptTurnTable::generateTimes()
{
  accel_time_ = (int)((max_angular_velocity/angular_acceleration)*1000+.5);
  const_time_ = (int)(((angle/max_angular_velocity)*1000+.5)-accel_time_);
  total_actual_time_ = 2 * accel_time_ + const_time_;
}
 
