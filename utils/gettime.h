#ifndef GETTIME_H
#define GETTIME_H

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

#endif//GETTIME_H
