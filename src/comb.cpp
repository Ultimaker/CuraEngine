/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "comb.h"

namespace cura {

bool Comb::preTest(Point startPoint, Point endPoint)
{
    return collisionTest(startPoint, endPoint);
}

bool Comb::collisionTest(Point startPoint, Point endPoint)
{
    Point diff = endPoint - startPoint;

    matrix = PointMatrix(diff);
    sp = matrix.apply(startPoint);
    ep = matrix.apply(endPoint);
    
    for(unsigned int n=0; n<boundary.size(); n++)
    {
        if (boundary[n].size() < 1)
            continue;
        Point p0 = matrix.apply(boundary[n][boundary[n].size()-1]);
        for(unsigned int i=0; i<boundary[n].size(); i++)
        {
            Point p1 = matrix.apply(boundary[n][i]);
            if ((p0.Y > sp.Y && p1.Y < sp.Y) || (p1.Y > sp.Y && p0.Y < sp.Y))
            {
                int64_t x = p0.X + (p1.X - p0.X) * (sp.Y - p0.Y) / (p1.Y - p0.Y);
                
                if (x > sp.X && x < ep.X)
                    return true;
            }
            p0 = p1;
        }
    }
    return false;
}

void Comb::calcMinMax()
{
    for(unsigned int n=0; n<boundary.size(); n++)
    {
        minX[n] = INT64_MAX;
        maxX[n] = INT64_MIN;
        Point p0 = matrix.apply(boundary[n][boundary[n].size()-1]);
        for(unsigned int i=0; i<boundary[n].size(); i++)
        {
            Point p1 = matrix.apply(boundary[n][i]);
            if ((p0.Y > sp.Y && p1.Y < sp.Y) || (p1.Y > sp.Y && p0.Y < sp.Y))
            {
                int64_t x = p0.X + (p1.X - p0.X) * (sp.Y - p0.Y) / (p1.Y - p0.Y);
                
                if (x >= sp.X && x <= ep.X)
                {
                    if (x < minX[n]) { minX[n] = x; minIdx[n] = i; }
                    if (x > maxX[n]) { maxX[n] = x; maxIdx[n] = i; }
                }
            }
            p0 = p1;
        }
    }
}

unsigned int Comb::getPolygonAbove(int64_t x)
{
    int64_t min = POINT_MAX;
    unsigned int ret = NO_INDEX;
    for(unsigned int n=0; n<boundary.size(); n++)
    {
        if (minX[n] > x && minX[n] < min)
        {
            min = minX[n];
            ret = n;
        }
    }
    return ret;
}

Point Comb::getBoundaryPointWithOffset(unsigned int polygonNr, unsigned int idx)
{
    Point p0 = boundary[polygonNr][(idx > 0) ? (idx - 1) : (boundary[polygonNr].size() - 1)];
    Point p1 = boundary[polygonNr][idx];
    Point p2 = boundary[polygonNr][(idx < (boundary[polygonNr].size() - 1)) ? (idx + 1) : (0)];
    
    Point off0 = crossZ(normal(p1 - p0, MM2INT(1.0)));
    Point off1 = crossZ(normal(p2 - p1, MM2INT(1.0)));
    Point n = normal(off0 + off1, MM2INT(0.2));
    
    return p1 + n;
}

Comb::Comb(Polygons& _boundary)
: boundary(_boundary)
{
    minX = new int64_t[boundary.size()];
    maxX = new int64_t[boundary.size()];
    minIdx = new unsigned int[boundary.size()];
    maxIdx = new unsigned int[boundary.size()];
}

Comb::~Comb()
{
    delete[] minX;
    delete[] maxX;
    delete[] minIdx;
    delete[] maxIdx;
}

bool Comb::moveInside(Point* p, int distance)
{
    Point ret = *p;
    int64_t bestDist = MM2INT(2.0) * MM2INT(2.0);
    for(unsigned int n=0; n<boundary.size(); n++)
    {
        if (boundary[n].size() < 1)
            continue;
        Point p0 = boundary[n][boundary[n].size()-1];
        for(unsigned int i=0; i<boundary[n].size(); i++)
        {
            Point p1 = boundary[n][i];
            
            //Q = A + Normal( B - A ) * ((( B - A ) dot ( P - A )) / VSize( A - B ));
            Point pDiff = p1 - p0;
            int64_t lineLength = vSize(pDiff);
            int64_t distOnLine = dot(pDiff, *p - p0) / lineLength;
            if (distOnLine < 10)
                distOnLine = 10;
            if (distOnLine > lineLength - 10)
                distOnLine = lineLength - 10;
            Point q = p0 + pDiff * distOnLine / lineLength;
            
            int64_t dist = vSize2(q - *p);
            if (dist < bestDist)
            {
                bestDist = dist;
                ret = q + crossZ(normal(p1 - p0, distance));
            }
            
            p0 = p1;
        }
    }
    if (bestDist < MM2INT(2.0) * MM2INT(2.0))
    {
        *p = ret;
        return true;
    }
    return false;
}

bool Comb::calc(Point startPoint, Point endPoint, std::vector<Point>& combPoints)
{
    if (shorterThen(endPoint - startPoint, MM2INT(1.5)))
        return true;
    
    bool addEndpoint = false;
    //Check if we are inside the comb boundaries
    if (!boundary.inside(startPoint))
    {
        if (!moveInside(&startPoint))    //If we fail to move the point inside the comb boundary we need to retract.
            return false;
        combPoints.push_back(startPoint);
    }
    if (!boundary.inside(endPoint))
    {
        if (!moveInside(&endPoint))    //If we fail to move the point inside the comb boundary we need to retract.
            return false;
        addEndpoint = true;
    }
    
    //Check if we are crossing any bounderies, and pre-calculate some values.
    if (!preTest(startPoint, endPoint))
    {
        //We're not crossing any boundaries. So skip the comb generation.
        if (!addEndpoint && combPoints.size() == 0) //Only skip if we didn't move the start and end point.
            return true;
    }
    
    //Calculate the minimum and maximum positions where we cross the comb boundary
    calcMinMax();
    
    int64_t x = sp.X;
    std::vector<Point> pointList;
    //Now walk trough the crossings, for every boundary we cross, find the initial cross point and the exit point. Then add all the points in between
    // to the pointList and continue with the next boundary we will cross, until there are no more boundaries to cross.
    // This gives a path from the start to finish curved around the holes that it encounters.
    while(true)
    {
        unsigned int n = getPolygonAbove(x);
        if (n == NO_INDEX) break;
        
        pointList.push_back(matrix.unapply(Point(minX[n] - MM2INT(0.2), sp.Y)));
        if ( (minIdx[n] - maxIdx[n] + boundary[n].size()) % boundary[n].size() > (maxIdx[n] - minIdx[n] + boundary[n].size()) % boundary[n].size())
        {
            for(unsigned int i=minIdx[n]; i != maxIdx[n]; i = (i < boundary[n].size() - 1) ? (i + 1) : (0))
            {
                pointList.push_back(getBoundaryPointWithOffset(n, i));
            }
        }else{
            if (minIdx[n] == 0)
                minIdx[n] = boundary[n].size() - 1;
            else
                minIdx[n]--;
            if (maxIdx[n] == 0)
                maxIdx[n] = boundary[n].size() - 1;
            else
                maxIdx[n]--;
            
            for(unsigned int i=minIdx[n]; i != maxIdx[n]; i = (i > 0) ? (i - 1) : (boundary[n].size() - 1))
            {
                pointList.push_back(getBoundaryPointWithOffset(n, i));
            }
        }
        pointList.push_back(matrix.unapply(Point(maxX[n] + MM2INT(0.2), sp.Y)));
        
        x = maxX[n];
    }
    pointList.push_back(endPoint);
    
    //Optimize the pointList, skip each point we could already reach by not crossing a boundary. This smooths out the path and makes it skip any unneeded corners.
    Point current_point = startPoint;
    for(unsigned int point_idx = 1; point_idx<pointList.size(); point_idx++)
    {
        if (collisionTest(current_point, pointList[point_idx]))
        {
            if (collisionTest(current_point, pointList[point_idx - 1]))
            {
                return false;
            }
            current_point = pointList[point_idx - 1];
            combPoints.push_back(current_point);
        }
    }
    if (addEndpoint)
        combPoints.push_back(endPoint);
    return true;
}

}//namespace cura
