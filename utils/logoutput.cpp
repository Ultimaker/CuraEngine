/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdio.h>
#include <stdarg.h>

#include "logoutput.h"

int verbose_level;

void logError(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fflush(stderr);
}

void _log(const char* fmt, ...)
{
    if (verbose_level < 1)
        return;

    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fflush(stderr);
}
void logProgress(const char* type, int value, int maxValue)
{
    if (verbose_level < 2)
        return;

    fprintf(stderr, "Progress:%s:%i:%i\n", type, value, maxValue);
    fflush(stderr);
}
