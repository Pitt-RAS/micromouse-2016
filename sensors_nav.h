#ifndef MICROMOUSE_SENSORS_NAV_H_
#define MICROMOUSE_SENSORS_NAV_H_

enum direction {n, ne, e, se, s, sw, w, nw};

bool nav_front_wall();
bool nav_left_wall();
bool nav_right_wall();
enum direction nav_direction();

#endif