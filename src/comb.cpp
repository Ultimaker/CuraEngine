/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "comb.h"

#include "debug.h"

namespace cura {

bool Comb::lineSegmentCollidesWithBoundary(Point startPoint, Point endPoint)
{
    Point diff = endPoint - startPoint;

    transformation_matrix = PointMatrix(diff);
    transformed_startPoint = transformation_matrix.apply(startPoint);
    transformed_endPoint = transformation_matrix.apply(endPoint);
    
    for(unsigned int n=0; n<boundary.size(); n++)
    {
        if (boundary[n].size() < 1)
            continue;
        Point p0 = transformation_matrix.apply(boundary[n][boundary[n].size()-1]);
        for(unsigned int i=0; i<boundary[n].size(); i++)
        {
            Point p1 = transformation_matrix.apply(boundary[n][i]);
            if ((p0.Y > transformed_startPoint.Y && p1.Y < transformed_startPoint.Y) || (p1.Y > transformed_startPoint.Y && p0.Y < transformed_startPoint.Y))
            {
                int64_t x = p0.X + (p1.X - p0.X) * (transformed_startPoint.Y - p0.Y) / (p1.Y - p0.Y);
                
                if (x > transformed_startPoint.X && x < transformed_endPoint.X)
                    return true;
            }
            p0 = p1;
        }
    }
    return false;
}

void Comb::calcMinMax()
{
    int64_t minX_global = INT64_MAX;
    int64_t maxX_global = INT64_MIN;
    for(unsigned int boundary_poly_idx = 0; boundary_poly_idx < boundary.size(); boundary_poly_idx++)
    {
        minX[boundary_poly_idx] = INT64_MAX;
        maxX[boundary_poly_idx] = INT64_MIN;
        PolygonRef poly = boundary[boundary_poly_idx];
        Point p0 = transformation_matrix.apply(poly.back());
        for(unsigned int boundary_point_idx = 0; boundary_point_idx < poly.size(); boundary_point_idx++)
        {
            Point p1 = transformation_matrix.apply(poly[boundary_point_idx]);
            if ((p0.Y > transformed_startPoint.Y && p1.Y < transformed_startPoint.Y) || (p1.Y > transformed_startPoint.Y && p0.Y < transformed_startPoint.Y))
            {
                int64_t x = p0.X + (p1.X - p0.X) * (transformed_startPoint.Y - p0.Y) / (p1.Y - p0.Y);
                
                if (x >= transformed_startPoint.X && x <= transformed_endPoint.X)
                {
                    if (x < minX[boundary_poly_idx]) { minX[boundary_poly_idx] = x; minIdx[boundary_poly_idx] = boundary_point_idx; }
                    if (x > maxX[boundary_poly_idx]) { maxX[boundary_poly_idx] = x; maxIdx[boundary_poly_idx] = boundary_point_idx; }
                    if (x < minX_global) { minX_global = x; minIdx_global = boundary_poly_idx; }
                    if (x > maxX_global) { maxX_global = x; maxIdx_global = boundary_poly_idx; }
                }
            }
            p0 = p1;
        }
    }
}

unsigned int Comb::getNextPolygonAlongScanline(int64_t x)
{
    int64_t min = POINT_MAX;
    unsigned int ret = NO_INDEX;
    for(unsigned int n = 0; n < boundary.size(); n++)
    {
        if (minX[n] > x && minX[n] < min)
        {
            min = minX[n];
            ret = n;
        }
    }
    return ret;
}

Point Comb::getBoundaryPointWithOffset(unsigned int polygon_idx, unsigned int point_idx)
{
    int64_t offset = MM2INT(0.2); // hard coded value
    PolygonRef poly = boundary[polygon_idx];
    Point p0 = poly[(point_idx > 0) ? (point_idx - 1) : (poly.size() - 1)];
    Point p1 = poly[point_idx];
    Point p2 = poly[(point_idx < (poly.size() - 1)) ? (point_idx + 1) : 0];
    
    Point off0 = crossZ(normal(p1 - p0, MM2INT(1.0))); // hard coded value (?)
    Point off1 = crossZ(normal(p2 - p1, MM2INT(1.0))); // hard coded value (?)
    Point n = normal(off0 + off1, offset);
    
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
    int64_t maxDist2 =  MM2INT(2.0) * MM2INT(2.0);
    int64_t bestDist2 = maxDist2;
    for(PolygonRef poly : boundary)
    {
        if (poly.size() < 1)
            continue;
        Point p0 = poly.back();
        for(Point& p1 : poly)
        {   
            //Q = A + Normal( B - A ) * ((( B - A ) dot ( P - A )) / VSize( A - B ));
            // A = p0
            // B = p1
            // Q = ret 
            // P = p
            Point pDiff = p1 - p0;
            int64_t lineLength = vSize(pDiff);
            int64_t distOnLine = dot(pDiff, *p - p0) / lineLength;
            if (distOnLine < 10)
                distOnLine = 10;
            if (distOnLine > lineLength - 10)
                distOnLine = lineLength - 10;
            Point q = p0 + pDiff * distOnLine / lineLength;
            
            int64_t dist2 = vSize2(q - *p);
            if (dist2 < bestDist2)
            {
                bestDist2 = dist2;
                ret = q + crossZ(normal(p1 - p0, distance));
            }
            
            p0 = p1;
        }
    }
    if (bestDist2 < maxDist2)
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
    
    //Check if we are crossing any boundaries, and pre-calculate some values.
    if (!lineSegmentCollidesWithBoundary(startPoint, endPoint))
    {
        //We're not crossing any boundaries. So skip the comb generation.
        if (!addEndpoint && combPoints.size() == 0) //Only skip if we didn't move the start and end point.
            return true;
    }
    
    //Calculate the minimum and maximum positions where we cross the comb boundary
    calcMinMax();
    
    std::vector<Point> pointList;
    getBasicCombingPath(endPoint, pointList);
    
    bool succeeded = optimizePath(startPoint, pointList, combPoints);
    if (addEndpoint)
        combPoints.push_back(endPoint);
    return succeeded;
}

bool Comb::calc(Point startPoint, Point endPoint, std::vector<std::vector<Point>>& combPaths)
{
    if (shorterThen(endPoint - startPoint, MM2INT(1.5)))
    {
//         DEBUG_PRINTLN("too short dist!");
        return true;
    }
    
    bool addEndpoint = false;
    //Check if we are inside the comb boundaries
    if (!boundary.inside(startPoint))
    {
        if (!moveInside(&startPoint))    //If we fail to move the point inside the comb boundary we need to retract.
        {   
//             std::cerr << " fail to move the start point inside the comb boundary we need to retract."<< std::endl;
            return false;
        }
        combPaths.emplace_back();
        combPaths.back().push_back(startPoint);
    }
    if (!boundary.inside(endPoint))
    {
        if (!moveInside(&endPoint))    //If we fail to move the point inside the comb boundary we need to retract.
        {
//             std::cerr << " fail to move the end point inside the comb boundary we need to retract."<< std::endl;
            return false;
        }
        addEndpoint = true;
    }
    
    
    //Check if we are crossing any boundaries, and pre-calculate some values.
    if (!lineSegmentCollidesWithBoundary(startPoint, endPoint))
    {
        //We're not crossing any boundaries. So skip the comb generation.
        if (!addEndpoint && combPaths.size() == 0) //Only skip if we didn't move the start and end point.
            return true;
    }
    
//     std::cerr << "calcuklating comb path!" << std::endl;
    
    //Calculate the minimum and maximum positions where we cross the comb boundary
    calcMinMax();
    
    std::vector<std::vector<Point>> basicCombPaths;
    getBasicCombingPaths(endPoint, basicCombPaths);
    
    bool succeeded = optimizePaths(startPoint, basicCombPaths, combPaths);
    if (addEndpoint)
        combPaths.back().push_back(endPoint);
    
//     std::cerr << "succeeded = " << succeeded << std::endl;
    return succeeded;
}

void Comb::getBasicCombingPath(Point endPoint, std::vector<Point>& pointList) 
{
    for (unsigned int poly_idx = getNextPolygonAlongScanline(transformed_startPoint.X); poly_idx != NO_INDEX; poly_idx = getNextPolygonAlongScanline(maxX[poly_idx]))
    {
        getBasicCombingPath(poly_idx, pointList);
    }
    pointList.push_back(endPoint);
}

void Comb::getBasicCombingPaths(Point endPoint, std::vector<std::vector<Point>>& combPaths) 
{
    for (unsigned int poly_idx = getNextPolygonAlongScanline(transformed_startPoint.X); poly_idx != NO_INDEX; poly_idx = getNextPolygonAlongScanline(maxX[poly_idx]))
    {
        combPaths.emplace_back();
        std::vector<Point>& pointList = combPaths.back();
        getBasicCombingPath(poly_idx, pointList);
    }
    if (combPaths.size() == 0)
    {
        combPaths.emplace_back();
    }
    combPaths.back().push_back(endPoint);
}

void Comb::getBasicCombingPath(unsigned int poly_idx, std::vector<Point>& pointList) 
{
    pointList.push_back(transformation_matrix.unapply(Point(minX[poly_idx] - MM2INT(0.2), transformed_startPoint.Y)));
    if ( (minIdx[poly_idx] - maxIdx[poly_idx] + boundary[poly_idx].size()) % boundary[poly_idx].size() > (maxIdx[poly_idx] - minIdx[poly_idx] + boundary[poly_idx].size()) % boundary[poly_idx].size())
    { // follow the path in the same direction as the winding order of the boundary polygon
        for(unsigned int point_idx = minIdx[poly_idx]; point_idx != maxIdx[poly_idx]; point_idx = (point_idx < boundary[poly_idx].size() - 1) ? (point_idx + 1) : (0))
        {
            pointList.push_back(getBoundaryPointWithOffset(poly_idx, point_idx));
        }
    }
    else
    {
        if (minIdx[poly_idx] == 0)
            minIdx[poly_idx] = boundary[poly_idx].size() - 1;
        else
            minIdx[poly_idx]--;
        if (maxIdx[poly_idx] == 0)
            maxIdx[poly_idx] = boundary[poly_idx].size() - 1;
        else
            maxIdx[poly_idx]--;
        
        for(unsigned int i=minIdx[poly_idx]; i != maxIdx[poly_idx]; i = (i > 0) ? (i - 1) : (boundary[poly_idx].size() - 1))
        {
            pointList.push_back(getBoundaryPointWithOffset(poly_idx, i));
        }
    }
    pointList.push_back(transformation_matrix.unapply(Point(maxX[poly_idx] + MM2INT(0.2), transformed_startPoint.Y)));
}

bool Comb::optimizePath(Point startPoint, std::vector<Point>& pointList, std::vector<Point>& combPoints) 
{
    
    Point current_point = startPoint;
    for(unsigned int point_idx = 1; point_idx<pointList.size(); point_idx++)
    {
        if (lineSegmentCollidesWithBoundary(current_point, pointList[point_idx]))
        {
            if (lineSegmentCollidesWithBoundary(current_point, pointList[point_idx - 1]))
            {
                return false;
            }
            current_point = pointList[point_idx - 1];
            combPoints.push_back(current_point);
        }
    }
    return true;
}

bool Comb::optimizePaths(Point startPoint, std::vector<std::vector<Point>>& basicCombPaths, std::vector<std::vector<Point>>& combPaths) 
{
    Point current_point = startPoint;
    bool first = true;
    for (std::vector<Point>& basicCombPath : basicCombPaths)
    {
        if (!first)
        {
            current_point = basicCombPath[0]; // TODO: don't cause the first point to get doubled
        }
        first = false;
        combPaths.emplace_back();
        std::vector<Point>& combPath = combPaths.back();
        
        bool succeeded = optimizePath(current_point, basicCombPath, combPath);
        if (!succeeded)
        {
            return false;
        }
    }
    return true;
}

}//namespace cura
