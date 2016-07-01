/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef GETTIME_H
#define GETTIME_H


#ifdef __WIN32
#include <windows.h>
#else
#include <sys/resource.h>
#include <sys/time.h>
#include <stddef.h>
#include <cassert>
#endif

namespace cura
{
static inline double getTime()
{
#ifdef __WIN32
    return double(GetTickCount()) / 1000.0;
#else
    int ret;
    struct rusage usage;
    ret = getrusage(RUSAGE_SELF,&usage);
    assert(ret==0);
    double user_time =  double(usage.ru_utime.tv_sec) + double(usage.ru_utime.tv_usec) / 1000000.0;
    double sys_time  =  double(usage.ru_stime.tv_sec) + double(usage.ru_stime.tv_usec) / 1000000.0;
    return user_time + sys_time;
                    
    //struct timeval tv;
    //gettimeofday(&tv, nullptr);
    //return double(tv.tv_sec) + double(tv.tv_usec) / 1000000.0;
#endif
}

class TimeKeeper
{
private:
    double startTime;
public:
    TimeKeeper();
    
    double restart();
};

}//namespace cura
#endif//GETTIME_H
