/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */

#ifndef MULTITHREAD_OPENMP_H
#define MULTITHREAD_OPENMP_H

#include <omp.h>

namespace cura
{

extern bool abort_execution;


inline bool checkMultithreadAbort()
{
    bool tmp_abort_execution;
#pragma omp atomic read
    tmp_abort_execution = abort_execution;
    return tmp_abort_execution;
}

inline void setMultithreadAbort()
{
#pragma omp atomic write
    abort_execution = true;
}

#ifdef _OPENMP
void handleMultithreadAbort();
#else
inline void handleMultithreadAbort(){}
#endif

#ifdef _OPENMP
#define MULTITHREAD_FOR_CATCH_EXCEPTION(code) \
    if (checkMultithreadAbort()) \
    { \
        continue; \
    } \
    try \
    { \
        code \
    } \
    catch (...) \
    { \
        setMultithreadAbort(); \
    }
#else
#define MULTITHREAD_FOR_CATCH_EXCEPTION(code) code
#endif

}//namespace cura

#endif // MULTITHREAD_OPENMP_H
