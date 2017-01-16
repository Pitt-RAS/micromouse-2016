#include "SweptTurnProfile.h"

#include <Arduino.h>
#include <cstddef>

// Dependencies within Micromouse
#include "conf.h"

SweptTurnProfile::SweptTurnProfile(float tangential_velocity,
                                   float turn_angle_deg,
                                   float time_step)
    : tangential_velocity_(tangential_velocity),
      turn_angle_(turn_angle_deg * DEG_TO_RAD),
      time_step_(time_step),
      inside_trigs_(ROBOT_MASS * (MM_BETWEEN_WHEELS / 2.0 / 1000.0)
                    * tangential_velocity_ / MOMENT_OF_INERTIA),
      omega0_(ROBOT_MASS * MAX_COEFFICIENT_FRICTION * 9.81 / ROBOT_MASS
              / tangential_velocity)
{
  // angle covered if we accelerate to max rotational velocity
  float angle_for_full_acceleration = omega0_ / inside_trigs_;

  if (angle_for_full_acceleration > turn_angle_ / 2) {
    // We don't have time to accelerate to max rotational velocity

    // Solve for t when theta = turn_angle / 2
    float temp = (turn_angle_ / 2) * inside_trigs_ / omega0_;
    acceleration_duration_ = acos(1 - temp) / inside_trigs_;

    const_velocity_duration_ = 0;
  } else {
    // We have time to accelerate to max rotational velocity

    // Solve for t when omega maxes out
    acceleration_duration_ = (PI / 2) / inside_trigs_;

    float theta_const = turn_angle_ - (getAngleAtTime(acceleration_duration_) * 2);
    const_velocity_duration_ = theta_const / getVelocityAtTime(acceleration_duration_);
  }

  turn_duration_ = 2 * acceleration_duration_ + const_velocity_duration_;

  for (size_t i = 0; i <= getTotalTurnSteps(); i++) {
    offset_table_[i] = getTurnOffset(getAngleAtTime(i * time_step));
  }
}

float SweptTurnProfile::getOffsetAtMicros(unsigned long input_time) const
{
  float fractional_index = (input_time / 1000000.0) / time_step_;
  int index = fractional_index;
  float low_offset = offset_table_[index];
  float high_offset = offset_table_[index+1];
  return (fractional_index - index) * (high_offset - low_offset) + low_offset;
}

size_t SweptTurnProfile::getTotalTurnSteps() const
{
  return (size_t)((turn_duration_) / time_step_ + 1);
}

float SweptTurnProfile::getAngleAtTime(float t) const
{
  if (t <= acceleration_duration_) {
    return (1 - cos(inside_trigs_ * t)) * omega0_ / inside_trigs_;
  } else if (t <= acceleration_duration_ + const_velocity_duration_) {
    return getAngleAtTime(acceleration_duration_)
         + getVelocityAtTime(acceleration_duration_) * t;
  } else {
    return turn_angle_ - getAngleAtTime(turn_duration_ - t);
  }
}

float SweptTurnProfile::getVelocityAtTime(float t) const
{
  return omega0_ * sin(inside_trigs_ * t);
}

float SweptTurnProfile::getTurnOffset(float angle) const
{
  return (MM_BETWEEN_WHEELS / 2.0) * angle;
}

unsigned long SweptTurnProfile::getTotalTime() const
{
  return turn_duration_ * 1000000;
}

float SweptTurnProfile::getTotalAngle() const
{
  return turn_angle_ * RAD_TO_DEG;
}
