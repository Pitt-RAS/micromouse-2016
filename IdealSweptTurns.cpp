#include "IdealSweptTurns.h"

#include <Arduino.h>

// Dependencies within Micromouse
#include "conf.h"

IdealSweptTurns::IdealSweptTurns(float temp_tangential_velocity, float temp_turn_angle, float temp_time_step)
{
  tangential_velocity = temp_tangential_velocity;
  turn_angle = temp_turn_angle * 3.14159265359 / 180.0;
  time_step = temp_time_step;
  frict_force = ROBOT_MASS * MAX_COEFFICIENT_FRICTION * 9.81;
  mm_per_radian = MM_BETWEEN_WHEELS / 2;

  // this is the constant that multiplies the time to get an argument for
  //   all of the trig functions
  inside_trigs = ROBOT_MASS * (MM_BETWEEN_WHEELS / 1000) * tangential_velocity;
  inside_trigs /= 2 * MOMENT_OF_INERTIA;

  // time duration of the acceleration phase
  acceleration_duration = MOMENT_OF_INERTIA * PI;
  acceleration_duration /= ROBOT_MASS * tangential_velocity * MM_BETWEEN_WHEELS / 1000;

  // angle the robot covers during the acceleration phase
  theta_accel = getAngleAtTime(acceleration_duration, true);

  // angle the robot covers during the zero-acceleration phase
  float theta_const;

  // Recalculate if we don't need all of our acceleration time
  if (theta_accel * 2 > turn_angle) {
    theta_const = 0.0;
    const_velocity_duration = 0.0;
    theta_accel = turn_angle / 2;

    // Solve for t from the equation for theta (when theta = theta_accel)
    acceleration_duration = theta_accel * (MM_BETWEEN_WHEELS / 1000)
                            * sq(tangential_velocity * ROBOT_MASS);
    acceleration_duration /= frict_force * MOMENT_OF_INERTIA;
    acceleration_duration = 1 - acos(acceleration_duration);

    max_velocity = getVelocityAtTime(acceleration_duration);

  } else {
    theta_const = turn_angle - (theta_accel * 2);
    const_velocity_duration = theta_const / getVelocityAtTime(acceleration_duration);
    max_velocity = getVelocityAtTime(acceleration_duration);
  }
  
  turn_duration = 2 * acceleration_duration + const_velocity_duration;

  //Serial.println((turn_duration/time_step) + 1);

  //offset_table = new float[getTotalTurnSteps()];

  int i = 0;
  for (i = 0; i <= getTotalTurnSteps(); i++) {
    if(i <= (int)((turn_duration / 2) / time_step + 1)){
      if (i * time_step <= acceleration_duration) {
        //Serial.println("acceleration step");
        offset_table[i] = getTurnOffset(getAngleAtTime(i * time_step, false));
      }
      else {
        //Serial.println("constant step1");
        offset_table[i] = getTurnOffset(theta_accel + max_velocity * (i * time_step - acceleration_duration));
      }    
    }else{
      //reverse order for second half of turn
      if(i* time_step <= (acceleration_duration + const_velocity_duration)){
        //Serial.println("constant step2");
        offset_table[i] = getTurnOffset(theta_accel + max_velocity * (i * time_step - acceleration_duration));
      }else{
        //Serial.println("dceleration step");
        offset_table[i] = getTurnOffset(getAngleAtTime(i * time_step, false));
      }
    }  
  }
}

float IdealSweptTurns::getOffsetAtMicros(unsigned long input_time)
{ 
  float fractional_index = (input_time / 1000000.0) / time_step;
  int index = fractional_index;
  float low_offset = offset_table[index];
  float high_offset = offset_table[index+1];
  //return low_offset;
  return (float)((fractional_index - index) * (high_offset - low_offset) + low_offset);
}

int IdealSweptTurns::getTotalTurnSteps()
{
  return (int)((turn_duration) / time_step + 1);
}

float IdealSweptTurns::getAngleAtTime(float t, bool build_time_table)
{
  if (t <= acceleration_duration || build_time_table) {
    float result = 2 * frict_force * MOMENT_OF_INERTIA;
    result /= sq(ROBOT_MASS * tangential_velocity) * (MM_BETWEEN_WHEELS / 1000);
    result *= 1 - cos(inside_trigs * t);
    return result;
  } else if (t <= acceleration_duration + const_velocity_duration) {
    return theta_accel + max_velocity * t;
  } else { //extrapolate the decceleration angles by taking the reverse of the acceleration
    return turn_angle - getAngleAtTime(turn_duration - t, true);
  }
}

float IdealSweptTurns::getVelocityAtTime(float t)
{
  float abs_sec_func = abs(1 / cos(inside_trigs * t));
  return (frict_force / (ROBOT_MASS * tangential_velocity)) * (tan(inside_trigs*t)  - tan(inside_trigs*t) * (-1 + abs_sec_func) / abs_sec_func);

  //return ((frict_force / (ROBOT_MASS * tangential_velocity)) * (tan(inside_trigs) - tan(inside_trigs) * (-1 + abs_sec_func) / abs_sec_func));
}

float IdealSweptTurns::getTurnOffset(float angle)
{
  //Serial.print("angle");
  //Serial.println(angle,5);
  return mm_per_radian * angle;
}

unsigned long IdealSweptTurns::getTotalTime()
{
  return turn_duration * 1000000;
}

float IdealSweptTurns::getTotalAngle()
{
  return turn_angle * RAD_TO_DEG;
}
