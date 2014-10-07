/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdio.h>
#include <stdarg.h>
#include <sys/time.h>

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
	logTimeStamp();
	
    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fflush(stderr);
}

void log(const char* fmt, ...)
{
    if (verbose_level < 1)
        return;

    logTimeStamp();

    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fflush(stderr);
}
void logProgress(const char* type, int value, int maxValue)
{
    if (!progressLogging)
        return;

    fprintf(stderr, "Progress:%s:%i:%i\n", type, value, maxValue);
    fflush(stderr);
}

void logTimeStamp()
{
	time_t td;
	struct tm* local;
	time(&td);		//Get the current time
	local=localtime(&td);
	fprintf(stderr, "(%04d/%02d/%02d %02d:%02d:%02d) ",(1900+local->tm_year),(1+local->tm_mon),
			local->tm_mday,local->tm_hour,local->tm_min,local->tm_sec);
	fflush(stderr);
}

}//namespace cura
