//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <stdio.h>
#include <stdarg.h>
#include <mutex>

#include "utils/logoutput.h"

namespace cura {

static int verbose_level;
static bool progressLogging;
static std::mutex log_mutex;

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
    va_list args;
    va_start(args, fmt);
    {
        std::unique_lock lock(log_mutex);
        fprintf(stderr, "[ERROR] ");
        vfprintf(stderr, fmt, args);
        fflush(stderr);
    }
    va_end(args);
}

void logWarning(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    {
        std::unique_lock lock(log_mutex);
        fprintf(stderr, "[WARNING] ");
        vfprintf(stderr, fmt, args);
        fflush(stderr);
    }
    va_end(args);
}

void logAlways(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    {
        std::unique_lock lock(log_mutex);
        vfprintf(stderr, fmt, args);
        fflush(stderr);
    }
    va_end(args);
}

void log(const char* fmt, ...)
{
    va_list args;
    if (verbose_level < 1)
        return;

    va_start(args, fmt);
    {
        std::unique_lock lock(log_mutex);
        vfprintf(stderr, fmt, args);
        fflush(stderr);
    }
    va_end(args);
}

void logDebug(const char* fmt, ...)
{
    va_list args;
    if (verbose_level < 2)
    {
        return;
    }
    va_start(args, fmt);
    {
        std::unique_lock lock(log_mutex);
        fprintf(stderr, "[DEBUG] ");
        vfprintf(stderr, fmt, args);
        fflush(stderr);
    }
    va_end(args);
}

void logProgress(const char* type, int value, int maxValue, float percent)
{
    if (!progressLogging)
        return;

    {
        std::unique_lock lock(log_mutex);
        fprintf(stderr, "Progress:%s:%i:%i \t%f%%\n", type, value, maxValue, percent);
        fflush(stderr);
    }
}

}//namespace cura
