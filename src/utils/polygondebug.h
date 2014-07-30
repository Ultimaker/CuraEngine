#ifndef POLYGON_DEBUG_H
#define POLYGON_DEBUG_H

#define DEBUG_SCALE 50.0

#include "polygon.h"
#include <unistd.h>
#include <stdio.h>

namespace cura {

static inline FILE* openDebug(const char* filename)
{
    FILE* f = fopen(filename, "w");
    fprintf(f, "<!DOCTYPE html><html><body>\n");
    
    fprintf(f, "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" style='width:%ipx;height:%ipx'>\n", 1024 * 16, 1024 * 16);
    fprintf(f, "<marker id='MidMarker' viewBox='0 0 10 10' refX='5' refY='5' markerUnits='strokeWidth' markerWidth='10' markerHeight='10' stroke='lightblue' stroke-width='2' fill='none' orient='auto'>");
    fprintf(f, "<path d='M 0 0 L 10 5 M 0 10 L 10 5'/>");
    fprintf(f, "</marker>");
    return f;
}

static inline void writeDebug(FILE* f, Polygons& polygons)
{
    fprintf(f, "<g fill-rule='evenodd' style=\"fill: gray; stroke:black;stroke-width:1\">\n");
    fprintf(f, "<path marker-mid='url(#MidMarker)' d=\"");
    for(unsigned int j=0; j<polygons.size(); j++)
    {
        PolygonRef p = polygons[j];
        for(unsigned int n=0; n<p.size(); n++)
        {
            if (n == 0)
                fprintf(f, "M");
            else
                fprintf(f, "L");
            fprintf(f, "%f,%f ", float(p[n].X)/DEBUG_SCALE, float(p[n].Y)/DEBUG_SCALE);
        }
        fprintf(f, "Z\n");
    }
    fprintf(f, "\"/>");
    fprintf(f, "</g>\n");
}

void closeDebug(FILE* f)
{
    fprintf(f, "</svg>\n");
    fprintf(f, "</body></html>");
    fclose(f);
}

}

#endif//POLYGON_DEBUG_H
