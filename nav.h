#ifndef MICROMOUSE_NAV_H_
#define MICROMOUSE_NAV_H_

struct pos {
	int x;
	int y;
};

void nav_search();
void nav_solve(struct pos target);
int nav_update();

#endif