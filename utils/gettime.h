/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef GETTIME_H
#define GETTIME_H

#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

static inline double getTime()
{
#ifdef WIN32
    return double(GetTickCount()) / 1000.0;
#else
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return double(tv.tv_sec) + double(tv.tv_usec) / 1000000.0;
#endif
}
double t_start = getTime();

static inline double timeElapsed(double &t,bool all_time = false) {
	double t_passed = getTime() - t;
	t = getTime();
	if(all_time)
		return getTime() - t_start;
	return t_passed;
}
#endif//GETTIME_H
