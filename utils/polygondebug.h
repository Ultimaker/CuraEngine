#ifndef POLYGON_DEBUG_H
#define POLYGON_DEBUG_H

#include <stdio.h>
#include "polygon.h"

class PolygonDebug
{
private:
    Polygons polys;
    const char* filename;
public:
    PolygonDebug(const char* filename)
    : filename(filename)
    {
    }
    
    PolygonDebug(const char* filename, Polygons polys)
    : filename(filename)
    {
        add(polys);
    }
    
    PolygonDebug& add(Polygons polys)
    {
        this->polys.add(polys);
        return *this;
    }
    
    ~PolygonDebug()
    {
        Point polyMin(INT_MAX, INT_MAX), polyMax(INT_MIN, INT_MIN);
        for(unsigned int j=0; j<polys.size(); j++)
        {
            for(unsigned int n=0; n<polys[j].size(); n++)
            {
                polyMin.X = std::min(polyMin.X, polys[j][n].X);
                polyMin.Y = std::min(polyMin.Y, polys[j][n].Y);
                polyMax.X = std::max(polyMax.X, polys[j][n].X);
                polyMax.Y = std::max(polyMax.Y, polys[j][n].Y);
            }
        }
        Point polySize = polyMax - polyMin;
    
        FILE* f = fopen(filename, "a");
        fprintf(f, "<!DOCTYPE html><html><body>\n");
        //for(unsigned int i=0; i<layers.size(); i++)
        //{
            float scale = std::max(polySize.X, polySize.Y) / 1500;
            fprintf(f, "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" style='width:%ipx;height:%ipx'>\n", int(polySize.X / scale), int(polySize.Y / scale));
            fprintf(f, "<g fill-rule='evenodd' style=\"fill: gray; stroke:black;stroke-width:1\">\n");
            fprintf(f, "<path d=\"");
            for(unsigned int j=0; j<polys.size(); j++)
            {
                PolygonRef p = polys[j];
                for(unsigned int n=0; n<p.size(); n++)
                {
                    if (n == 0)
                        fprintf(f, "M");
                    else
                        fprintf(f, "L");
                    fprintf(f, "%f,%f ", float(p[n].X - polyMin.X)/scale, float(p[n].Y - polyMin.Y)/scale);
                }
                fprintf(f, "Z\n");
            }
            fprintf(f, "\"/>");
            fprintf(f, "</g>\n");
            fprintf(f, "</svg>\n");
        //}
        fprintf(f, "</body></html>");
        fclose(f);
    }
};

#endif//POLYGON_DEBUG_H
