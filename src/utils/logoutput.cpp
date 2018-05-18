/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdio.h>
#include <stdarg.h>

#include "logoutput.h"

namespace cura {

static int verbose_level;
static bool progressLogging;

const unsigned int LOG_MSG_SIZE = 1024* 1024;

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
#ifdef USE_G3LOG
    char buff[cura::LOG_MSG_SIZE];
    va_list args;
    va_start(args, fmt);
    vsprintf(buff, fmt, args);
    va_end(args);
    LOG(WARNING) << buff;
#else
    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fflush(stderr);
#endif
}

void log(const char* fmt, ...)
{
#ifdef USE_G3LOG
    char buff[cura::LOG_MSG_SIZE];
    va_list args;
    va_start(args, fmt);
    vsprintf(buff, fmt, args);
    va_end(args);
    LOG(INFO) << buff;
#else
    if (verbose_level < 1)
        return;

    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fflush(stderr);
#endif
}
void logProgress(const char* type, int value, int maxValue)
{
#ifdef USE_G3LOG
    char buff[cura::LOG_MSG_SIZE];
    sprintf(buff, "Progress:%s:%i:%i\n", type, value, maxValue);
    LOG(INFO) << buff;
#else
    if (!progressLogging)
        return;

    fprintf(stderr, "Progress:%s:%i:%i\n", type, value, maxValue);
    fflush(stderr);
#endif
}

}//namespace cura
