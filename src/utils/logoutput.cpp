/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdio.h>
#include <stdarg.h>

#ifdef _OPENMP
    #include <omp.h>
#endif // _OPENMP
#include "logoutput.h"

namespace cura {

static int verbose_level;
static bool progressLogging;

void increaseVerboseLevel()
{
    verbose_level++;
}

void enableProgressLogging()
{
    progressLogging = true;
}

void logError(const char* fmt, ...)
{
    #pragma omp critical
    {
        va_list args;
        va_start(args, fmt);
        fprintf(stderr, "[ERROR] ");
        vfprintf(stderr, fmt, args);
        va_end(args);
        fflush(stderr);
    }
}

void logWarning(const char* fmt, ...)
{
    #pragma omp critical
    {
        va_list args;
        va_start(args, fmt);
        fprintf(stderr, "[WARNING] ");
        vfprintf(stderr, fmt, args);
        va_end(args);
        fflush(stderr);
    }
}

void logAlways(const char* fmt, ...)
{
    #pragma omp critical
    {
        va_list args;
        va_start(args, fmt);
        vfprintf(stderr, fmt, args);
        va_end(args);
        fflush(stderr);
    }
}

void log(const char* fmt, ...)
{
    if (verbose_level < 1)
        return;

    #pragma omp critical
    {
        va_list args;
        va_start(args, fmt);
        vfprintf(stderr, fmt, args);
        va_end(args);
        fflush(stderr);
    }
}

void logDebug(const char* fmt, ...)
{
    if (verbose_level < 2)
    {
        return;
    }
    #pragma omp critical
    {
        va_list args;
        va_start(args, fmt);
        fprintf(stderr, "[DEBUG] ");
        vfprintf(stderr, fmt, args);
        va_end(args);
        fflush(stderr);
    }
}

void logProgress(const char* type, int value, int maxValue, float percent)
{
    if (!progressLogging)
        return;

    #pragma omp critical
    {
        fprintf(stderr, "Progress:%s:%i:%i \t%f%%\n", type, value, maxValue, percent);
        fflush(stderr);
    }
}

}//namespace cura
