void motion_set_max_speed(float new_max_speed);
void motion_set_max_accel(float new_max_accel);

void motion_forward(float distance, float exit_speed);
void motion_rotate(float angle);
void motion_corner(float angle, float radius, float exit_speed);

void motion_hold(int time);
