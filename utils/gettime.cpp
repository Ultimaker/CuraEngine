/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "gettime.h"

static double t_start = getTime();

double timeElapsed(double &t, bool all_time)
{
	double t_passed = getTime() - t;
	t = getTime();
	if(all_time)
		return getTime() - t_start;
	return t_passed;
}
