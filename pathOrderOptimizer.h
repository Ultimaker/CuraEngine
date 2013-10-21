/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef PATHOPTIMIZER_H
#define PATHOPTIMIZER_H

#include <stdint.h>
#include "utils/polygon.h"

class PathOrderOptimizer
{
public:
    Point startPoint;
    vector<ClipperLib::Polygon*> polygons;
    vector<int> polyStart;
    vector<int> polyOrder;

    PathOrderOptimizer(ClipperLib::IntPoint startPoint)
    {
        this->startPoint = startPoint;
    }

    void addPolygon(ClipperLib::Polygon& polygon)
    {
        this->polygons.push_back(&polygon);
    }
    
    void addPolygons(ClipperLib::Polygons& polygons)
    {
        for(unsigned int i=0;i<polygons.size(); i++)
            this->polygons.push_back(&polygons[i]);
    }
    
    void optimize();
};

#endif//PATHOPTIMIZER_H
