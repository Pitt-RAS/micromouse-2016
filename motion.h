#ifndef MICROMOUSE_MOTION_H_
#define MICROMOUSE_MOTION_H_

void motion_set_max_speed(float new_max_speed);
void motion_set_max_accel(float new_max_accel);

void motion_forward(float distance, float exit_speed);
void motion_rotate(float angle);
void motion_corner(float angle, float radius, float exit_speed);

void motion_hold(int time);

// functions to set max velocity variables
void motion_set_maxAccel_straight(float a);
void motion_set_maxDecel_straight(float a);
void motion_set_maxAccel_rotate(float a);
void motion_set_maxDecel_rotate(float a);
void motion_set_maxAccel_corner(float a);
void motion_set_maxDecel_corner(float a);
void motion_set_maxVel_straight(float a);
void motion_set_maxVel_rotate(float a);
void motion_set_maxVel_corner(float a);

#endif
