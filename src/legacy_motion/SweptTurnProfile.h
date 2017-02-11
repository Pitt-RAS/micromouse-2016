#ifndef SWEPT_TURN_PROFILE_H
#define SWEPT_TURN_PROFILE_H

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

  const float inside_trigs_;
  const float omega_max_;

  const float acceleration_duration_;
  const float const_velocity_duration_;
  const float turn_duration_;

 public:
  SweptTurnProfile(float tangential_velocity, float turn_angle);

  float getAngle(float t) const;
  float getAngularAcceleration(float t) const;
  float getAngularVelocity(float t) const;
  float getTotalAngle() const;
  float getTotalTime() const;
};

#endif
