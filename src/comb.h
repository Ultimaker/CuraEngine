/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef COMB_H
#define COMB_H

#include "utils/polygon.h"

namespace cura {

class Comb
{
private:
    Polygons& boundery;

    int64_t* minX;
    int64_t* maxX;
    unsigned int* minIdx;
    unsigned int* maxIdx;

    PointMatrix matrix;
    Point sp;
    Point ep;

    bool preTest(Point startPoint, Point endPoint);    
    bool collisionTest(Point startPoint, Point endPoint);

    void calcMinMax();
    
    unsigned int getPolygonAbove(int64_t x);
    
    Point getBounderyPointWithOffset(unsigned int polygonNr, unsigned int idx);
    
public:
    Comb(Polygons& _boundery);
    ~Comb();
    
    bool inside(const Point p) { return boundery.inside(p); }
    bool moveInside(Point* p, int distance = 100);
    
    bool calc(Point startPoint, Point endPoint, vector<Point>& combPoints);
};

}//namespace cura

#endif//COMB_H
