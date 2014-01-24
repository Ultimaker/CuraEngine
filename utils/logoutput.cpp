/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdio.h>
#include <stdarg.h>

#include "utils/logoutput.h"

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

void logPolygons(const char* name, int layerNr, int z, Polygons& polygons)
{
    if (verbose_level < 3)
        return;
    
    fprintf(stderr, "Polygons:%s:%i:%i:%f\n", name, layerNr, polygons.size(), float(z) / 1000);
    for(unsigned int n=0; n<polygons.size(); n++)
    {
        for(unsigned int i=0; i<polygons[n].size(); i++)
            fprintf(stderr, "%.2f %.2f ", float(polygons[n][i].X) / 1000, float(polygons[n][i].Y) / 1000);
        fprintf(stderr, "\n");
    }
}
