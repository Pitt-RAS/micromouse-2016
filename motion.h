#ifndef MICROMOUSE_MOTION_H_
#define MICROMOUSE_MOTION_H_

// Dependencies within Micromouse
#include "IdealSweptTurns.h"

void motion_set_max_speed(float new_max_speed);
void motion_set_max_accel(float new_max_accel);

void motion_forward(float distance, float current_speed, float exit_speed);
void motion_forward_diag(float distance, float current_speed, float exit_speed);
void motion_collect(float distance, float current_speed, float exit_speed);
void motion_rotate(float angle);
void motion_gyro_rotate(float angle);
void motion_corner(SweptTurnType turn_type, float speed, float size_scaling = 1);

void motion_hold(unsigned int time);
void motion_hold_range(int setpoint, unsigned int time);

// functions to set max velocity variables
void motion_set_maxAccel_straight(float temp_max_accel_straight);
void motion_set_maxDecel_straight(float temp_max_decel_straight);
void motion_set_maxAccel_rotate(float temp_max_accel_rotate);
void motion_set_maxDecel_rotate(float temp_max_decel_rotate);
void motion_set_maxAccel_corner(float temp_max_accel_corner);
void motion_set_maxDecel_corner(float temp_max_decel_corner);
void motion_set_maxVel_straight(float temp_max_vel_straight);
void motion_set_maxVel_diag(float temp_max_vel_diag);
void motion_set_maxVel_rotate(float temp_max_vel_rotate);
void motion_set_maxVel_corner(float temp_max_vel_corner);
float motion_get_maxAccel_straight();
float motion_get_maxDecel_straight();
float motion_get_maxAccel_rotate();
float motion_get_maxDecel_rotate();
float motion_get_maxAccel_corner();
float motion_get_maxDecel_corner();
float motion_get_maxVel_straight();
float motion_get_maxVel_diag();
float motion_get_maxVel_rotate();
float motion_get_maxVel_corner();

#endif
