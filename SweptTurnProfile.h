#ifndef IDEALSWEPTTURNS_H
#define IDEALSWEPTTURNS_H

#include <cstddef>

enum SweptTurnType {
  kLeftTurn45, kLeftTurn90, kLeftTurn135, kLeftTurn180,
  kRightTurn45, kRightTurn90, kRightTurn135, kRightTurn180
};

class SweptTurnProfile {
 private:
  // All internal variables in mks units unless stated otherwise
  const float tangential_velocity_;
  const float turn_angle_;
  const float time_step_;

  const float inside_trigs_;
  const float omega_max_;

  const float acceleration_duration_;
  const float const_velocity_duration_;
  const float turn_duration_;

  // Offsets in mm for wheels at each timestep
  float offset_table_[500];

  float getAngleAtTime(float t) const;
  size_t getTotalTurnSteps() const;
  float getTurnOffset(float angle) const;
  float getVelocityAtTime(float t) const;

 public:
  SweptTurnProfile(float tangential_velocity,
                   float turn_angle,
                   float time_step);

  float getOffsetAtMicros(unsigned long t) const;
  unsigned long getTotalTime() const;
  float getTotalAngle() const;
};

#endif
