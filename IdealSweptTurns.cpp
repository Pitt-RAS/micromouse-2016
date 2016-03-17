#include "Arduino.h"

#include "conf.h"
#include "IdealSweptTurns.h"

IdealSweptTurns::IdealSweptTurns(float temp_tangential_velocity, float temp_turn_angle, float temp_time_step)
{
  tangential_velocity = temp_tangential_velocity;
  turn_angle = temp_turn_angle * 3.14159265359 / 180.0;
  time_step = temp_time_step;
  frict_force = ROBOT_MASS * MAX_COEFFICIENT_FRICTION * 9.81;
  inside_trigs = (ROBOT_MASS * (MM_BETWEEN_WHEELS / 1000) * tangential_velocity) / (2 * MOMENT_OF_INERTIA);
  mm_per_degree = 3.14159265359 * MM_BETWEEN_WHEELS / 360;

  int i = 0;
  //get the max acceleration duration by finding the time that the angle begins to get smaller
  while (getAngleAtTime(i * time_step) < getAngleAtTime((i + 1) * time_step)) {
    i++;
  }

  acceleration_duration = i * time_step;
  float theta_accel = getAngleAtTime(acceleration_duration);
  float theta_const = turn_angle - theta_accel * 2;
  
  const_velocity_duration = theta_const / getVelocityAtTime(acceleration_duration);
  float max_velocity = getVelocityAtTime(acceleration_duration);


  turn_duration = 2 * acceleration_duration + const_velocity_duration;

  for (i = 0; i <= (int)((turn_duration) / time_step + 1); i++) {
    if(i <= (int)((turn_duration / 2) / time_step + 1)){
      if (i * time_step <= acceleration_duration) {
        offset_table[i] = getTurnOffset(getAngleAtTime(i * time_step));
      }
      else {
        offset_table[i] = getTurnOffset(theta_accel + max_velocity * (i * time_step - acceleration_duration));
      }    
    }else{
      //reverse order for second half of turn
      if(i* time_step <= (acceleration_duration + const_velocity_duration)){
        offset_table[i] = getTurnOffset(theta_accel + max_velocity * (i * time_step - acceleration_duration));
      }else{
        offset_table[i] = getTurnOffset(getAngleAtTime(i * time_step));
      }
    }  
  }
}

float IdealSweptTurns::getOffsetAtMicros(unsigned long input_time)
{
  int index = input_time / (1000000.0 * time_step);
  float low_theta = offset_table[index];
  float high_theta = offset_table[index+1];
  return (float)((((input_time / 1000000.0) - (float)index * time_step)/time_step)*(high_theta - low_theta));
}

float IdealSweptTurns::getAngleAtTime(float t)
{
  float return_angle = 0;
  float abs_sec_func = abs(1 / cos(inside_trigs * t));
  if(t <= acceleration_duration + const_velocity_duration)
    return (((2.0 * frict_force * MOMENT_OF_INERTIA) / (pow(ROBOT_MASS * tangential_velocity, 2) * (MM_BETWEEN_WHEELS / 1000))) * (-1.0 + abs_sec_func) / (abs_sec_func));
  else //extrapolate the decceleration angles by taking the reverse of the acceleration
    return turn_angle - getAngleAtTime(turn_duration - t);
}

float IdealSweptTurns::getVelocityAtTime(float t)
{
  float abs_sec_func = abs(1 / cos(inside_trigs * t));

  return ((frict_force / (ROBOT_MASS * tangential_velocity)) * (tan(inside_trigs) - tan(inside_trigs) * (-1 + abs_sec_func) / abs_sec_func));
}

float IdealSweptTurns::getTurnOffset(float angle)
{
  return mm_per_degree * angle;
}
