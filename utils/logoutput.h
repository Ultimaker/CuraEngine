#ifndef LOGOUTPUT_H
#define LOGOUTPUT_H

extern int verbose_level;

void logError(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vfprintf(stdout, fmt, args);
    va_end(args);
    fflush(stdout);
}

void _log(const char* fmt, ...)
{
    if (verbose_level < 1)
        return;

    va_list args;
    va_start(args, fmt);
    vfprintf(stdout, fmt, args);
    va_end(args);
    fflush(stdout);
}
void logProgress(const char* type, int value, int maxValue)
{
    if (verbose_level < 2)
        return;

    fprintf(stdout, "Progress:%s:%i:%i\n", type, value, maxValue);
    fflush(stdout);
}
#define log _log

#endif//LOGOUTPUT_H
