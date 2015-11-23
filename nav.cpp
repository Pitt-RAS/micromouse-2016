#include "nav.h"

int searching = 0;
int solving = 0;

void search();

void nav_search()
{
	searching = 1;
}

void nav_solve(struct pos target)
{
}

int nav_update()
{
	if (searching) search();
}

void search()
{
}
