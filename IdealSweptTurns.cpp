#include "Arduino.h"

#include "conf.h"
#include "IdealSweptTurns.h"
 
IdealSweptTurns::IdealSweptTurns(float temp_tangential_velocity, float temp_turn_angle, float temp_time_step)
{
  tangential_velocity = temp_tangential_velocity;
  turn_angle = temp_turn_angle;
  time_step = temp_time_step;


  int i = 0;
  while(getAngleAtTime(i*time_step) < getAngleAtTime(i*time_step+1*time_step)){
    i++;
  }

  acceleration_duration = i*time_step;
  
  float theta_const = turn_angle - getAngleAtTime(acceleration_duration)*2;
  const_velocity_duration = theta_const/getVelocityAtTime(acceleration_duration);

  turn_duration = 2*acceleration_duration + const_velocity_duration;

  for(i = 1; i<(int)(turn_duration/time_step + .5);i++){
    offset_table[i] = getTurnOffset(getAngleAtTime(i*time_step));
  }
}

float IdealSweptTurns::getAngleAtTime(float t)
{

  float frict_force = ROBOT_MASS * MAX_COEFFICIENT_FRICTION * 9.81;
  float inside_trigs = (ROBOT_MASS * (t * MM_BETWEEN_WHEELS *tangential_velocity) / (2* MOMENT_OF_INERTIA)); 
  float abs_sec_func = abs(1/cos(inside_trigs));

  float theta = ((2.0* frict_force*MOMENT_OF_INERTIA)/(pow(ROBOT_MASS,2) *pow(tangential_velocity,2) * MM_BETWEEN_WHEELS))*(-1.0+abs_sec_func)/(abs_sec_func);
  return theta;
}

float IdealSweptTurns::getVelocityAtTime(float t)
{
  float frict_force = ROBOT_MASS * MAX_COEFFICIENT_FRICTION * 9.81;
  float inside_trigs = (ROBOT_MASS * (t * MM_BETWEEN_WHEELS * tangential_velocity) / (2 * MOMENT_OF_INERTIA)); 
  float abs_sec_func = abs(1/cos(inside_trigs));

  float theta_prime = (frict_force /(ROBOT_MASS * tangential_velocity)) * (tan(inside_trigs) - tan(inside_trigs) * (-1+abs_sec_func) / abs_sec_func);

  return theta_prime;
 
}

float IdealSweptTurns::getTurnOffset(float angle)
{
  float distance_per_degree = 3.14159265359 * MM_BETWEEN_WHEELS / 360;
  float rotation_offset = distance_per_degree * angle;
  return rotation_offset;
}
