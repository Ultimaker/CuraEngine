/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "comb.h"

#include <algorithm>

#include "debug.h"
#include "utils/polygonUtils.h"

namespace cura {
/*
bool Comb::lineSegmentCollidesWithAnyInsideBoundary(Point startPoint, Point endPoint)
{
    Point diff = endPoint - startPoint;

    transformation_matrix = PointMatrix(diff);
    transformed_startPoint = transformation_matrix.apply(startPoint);
    transformed_endPoint = transformation_matrix.apply(endPoint);
    
    for(PolygonsPart& part : parts_inside)
    {
        if (part.size() < 1)
            continue;
        Point p0 = transformation_matrix.apply(part.back());
        for(PolygonRef poly : part)
        {
            Point p1 = transformation_matrix.apply(poly);
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
}*/

bool LinePolygonsCrossings::lineSegmentCollidesWithBoundary()
{
    Point diff = endPoint - startPoint;

    transformation_matrix = PointMatrix(diff);
    transformed_startPoint = transformation_matrix.apply(startPoint);
    transformed_endPoint = transformation_matrix.apply(endPoint);

    for(PolygonRef poly : boundary)
    {
        Point p0 = transformation_matrix.apply(poly.back());
        for(Point p1_ : poly)
        {
            Point p1 = transformation_matrix.apply(p1_);
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
/*
bool Comb::lineSegmentCollidesWithInsideBoundary(Point startPoint, Point endPoint, PolygonsPart part)
{
    Point diff = endPoint - startPoint;

    PointMatrix transformation_matrix = PointMatrix(diff);
    Point transformed_startPoint = transformation_matrix.apply(startPoint);
    Point transformed_endPoint = transformation_matrix.apply(endPoint);


    Point p0 = transformation_matrix.apply(part.back());
    for(unsigned int i=0; i<part.size(); i++)
    {
        Point p1 = transformation_matrix.apply(part[i]);
        if ((p0.Y > transformed_startPoint.Y && p1.Y < transformed_startPoint.Y) || (p1.Y > transformed_startPoint.Y && p0.Y < transformed_startPoint.Y))
        {
            int64_t x = p0.X + (p1.X - p0.X) * (transformed_startPoint.Y - p0.Y) / (p1.Y - p0.Y);
            
            if (x > transformed_startPoint.X && x < transformed_endPoint.X)
                return true;
        }
        p0 = p1;
    }
    
    return false;
}

bool Comb::lineSegmentCollidesWithAnyOutsideBoundary(Point startPoint, Point endPoint)
{
    Point diff = endPoint - startPoint;

    PointMatrix transformation_matrix = PointMatrix(diff);
    Point transformed_startPoint = transformation_matrix.apply(startPoint);
    Point transformed_endPoint = transformation_matrix.apply(endPoint);
    
    for(PolygonsPart& part : parts_outside)
    {
        if (part.size() < 1)
            continue;
        Point p0 = transformation_matrix.apply(part.back());
        for(PolygonRef poly : part)
        {
            Point p1 = transformation_matrix.apply(poly);
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
}*/
/*
Point Comb::getBoundaryPointWithOffset(unsigned int polygon_idx, unsigned int point_idx, int64_t offset)
{
    PolygonRef poly = parts_inside[polygon_idx];
    Point p0 = poly[(point_idx > 0) ? (point_idx - 1) : (poly.size() - 1)];
    Point p1 = poly[point_idx];
    Point p2 = poly[(point_idx < (poly.size() - 1)) ? (point_idx + 1) : 0];
    
    Point off0 = crossZ(normal(p1 - p0, MM2INT(1.0))); // 1.0 determines precision only ...
    Point off1 = crossZ(normal(p2 - p1, MM2INT(1.0))); // ... normals will be normalized again here \/
    Point n = normal(off0 + off1, offset);
    
    return p1 + n;
}*/

Comb::Comb(Polygons& _boundary)
: boundary(_boundary)
{
    int64_t offset = MM2INT(2);
//     boundary_inside = boundary.offset(-offset);
//     boundary_outside = boundary.offset(offset);
//     
//     parts_inside = boundary_inside.splitIntoParts();
//     parts_outside = boundary_outside.splitIntoParts();
//     
//     crossings_inside = new PartCrossings[parts_inside.size()];
//     crossings_outside = new PartCrossings[parts_outside.size()];
}

Comb::~Comb()
{
//     delete[] crossings_inside;
//     delete[] crossings_outside;
}

unsigned int Comb::moveInside(Point* from, int distance)
{
    Point ret = *from;
    int64_t maxDist2 =  MM2INT(2.0) * MM2INT(2.0) * 2;
    int64_t bestDist2 = maxDist2;
    unsigned int bestPoly = NO_INDEX;
    for (unsigned int poly_idx = 0; poly_idx < boundary_inside.size(); poly_idx++)
    {
        PolygonRef poly = boundary_inside[poly_idx];
        if (poly.size() < 2)
            continue;
        Point p0 = poly[poly.size()-2];
        Point p1 = poly.back();
        bool projected_p_beyond_prev_segment = dot(p1 - p0, *from - p0) > vSize2(p1 - p0);
        for(Point& p2 : poly)
        {   
            // X = A + Normal( B - A ) * ((( B - A ) dot ( P - A )) / VSize( A - B ));
            // X = P projected on AB
            Point& a = p1;
            Point& b = p2;
            Point& p = *from;
            Point ab = b - a;
            Point ap = p - a;
            int64_t ab_length = vSize(ab);
            int64_t ax_length = dot(ab, ap) / ab_length;
            if (ax_length < 0) // x is projected to before ab
            {
                if (projected_p_beyond_prev_segment)
                { //  case which looks like:   > .
                    projected_p_beyond_prev_segment = false;
                    Point& x = p1;
                    
                    int64_t dist2 = vSize2(x - p);
                    if (dist2 < bestDist2)
                    {
                        bestDist2 = dist2;
                        ret = x + normal(crossZ(normal(a, distance*4) + normal(p1 - p0, distance*4)), distance); // *4 to retain more precision for the eventual normalization
                        bestPoly = poly_idx;
                    }
                }
                else
                {
                    projected_p_beyond_prev_segment = false;
                    p0 = p1;
                    p1 = p2;
                    continue;
                }
            }
            else if (ax_length > ab_length) // x is projected to beyond ab
            {
                projected_p_beyond_prev_segment = true;
                p0 = p1;
                p1 = p2;
                continue;
            }
            else 
            {
                projected_p_beyond_prev_segment = false;
                Point x = a + ab * ax_length / ab_length;
                
                int64_t dist2 = vSize2(x - *from);
                if (dist2 < bestDist2)
                {
                    bestDist2 = dist2;
                    ret = x + crossZ(normal(ab, distance));
                    bestPoly = poly_idx;
                }
            }
            
            
            p0 = p1;
            p1 = p2;
        }
    }
    if (bestDist2 < maxDist2)
    {
        *from = ret;
        return bestPoly;
    }
    return NO_INDEX;
}

bool Comb::calc(Point startPoint, Point endPoint, CombPaths& combPaths)
{
    if (shorterThen(endPoint - startPoint, MM2INT(1.5)))
    {
//         DEBUG_PRINTLN("too short dist!");
        return true;
    }
    
    DEBUG_PRINTLN(" >>>>>>>>>>>>>>>>>>>>>> ");
    int64_t offset = MM2INT(2);
    
    CombPaths basicCombPaths;
    basicCombPaths.emplace_back();
    
    boundary_inside = boundary.offset(-offset);
    std::vector<std::vector<unsigned int>> partsView = boundary_inside.splitIntoPartsView(); // !! changed the order of boundary_inside 
   
      

    DEBUG_SHOW(partsView[0][0]);
    for (int i = 0; i < partsView[0].size(); i++)
        DEBUG_SHOW(i<<":"<<partsView[0][i]);
    
    for (int i = 0; i < boundary.size(); i++)
    {
        DEBUG_SHOW(boundary[i].size());
    }
    
    
    bool addEndpoint = false;

    
    //Check if we are inside the comb boundaries
    unsigned int start_inside_poly = boundary_inside.findInside(startPoint, true);
    if (start_inside_poly == NO_INDEX)
    {
        DEBUG_PRINTLN("moving start point");
        start_inside_poly = moveInside(&startPoint);
        if (start_inside_poly == NO_INDEX)    //If we fail to move the point inside the comb boundary we need to retract.
        {   
            std::cerr << " fail to move the start point inside the comb boundary we need to retract. <<<<<<<<<<<< "<< std::endl;
            return false;
        }
        basicCombPaths.back().push_back(startPoint);
    }
    unsigned int end_inside_poly = boundary_inside.findInside(endPoint, true);
    if (end_inside_poly == NO_INDEX)
    {
        DEBUG_PRINTLN("moving end point");
        end_inside_poly = moveInside(&endPoint);
        if (end_inside_poly == NO_INDEX)    //If we fail to move the point inside the comb boundary we need to retract.
        {
            std::cerr << " fail to move the end point inside the comb boundary we need to retract. <<<<<<<<<<<< "<< std::endl;
            return false;
        }
        addEndpoint = true;
    }
  
  
    auto assemblePartContaining = [&](unsigned int poly_idx, unsigned int& boundary_poly_idx, Polygons& polygons) // TODO: move to polygon.cpp
    {
        PolygonsPart ret;
        for (unsigned int part_idx_now = 0; part_idx_now < partsView.size(); part_idx_now++)
        {
            std::vector<unsigned int>& partView = partsView[part_idx_now];
            if (partView.size() == 0) { continue; }
            std::vector<unsigned int>::iterator result = std::find(partView.begin(), partView.end(), poly_idx);
            if (result != partView.end()) 
            { 
                boundary_poly_idx = partView[0];
                for (unsigned int poly_idx_ff : partView)
                {
                    ret.add(polygons[poly_idx]);
                }
            }
        }
        return ret;
    };
    
//     std::cerr << "calculating comb path!" << std::endl;
    DEBUG_SHOW(boundary_inside.size());
    DEBUG_SHOW(start_inside_poly);
    DEBUG_SHOW(end_inside_poly);
    if (start_inside_poly == end_inside_poly)
    { // normal combing within part
        PolygonsPart part = assemblePartContaining(start_inside_poly, start_inside_poly, boundary_inside);
        
        LinePolygonsCrossings linePolygonsCrossings(part, startPoint, endPoint);
        if (!linePolygonsCrossings.lineSegmentCollidesWithBoundary())
        {
            //We're not crossing any boundaries. So skip the comb generation.
            if (!addEndpoint && basicCombPaths.size() == 0) //Only skip if we didn't move the start and end point.
            {
                DEBUG_PRINTLN("combing can go straight without problem! <<<<<<<<<<<< ");
                return true;
            }
            
        }
        linePolygonsCrossings.calcScanlineCrossings();
        
        linePolygonsCrossings.getBasicCombingPathPart(endPoint, basicCombPaths.back());
        
        combPaths.emplace_back();
        Polygons offsettedBoundary = boundary.offset(-(offset - 20));
        optimizePath(offsettedBoundary, startPoint, basicCombPaths.back(), combPaths.back());
    }
    else 
    { // comb to edge >> move through air avoiding other parts >> comb inside end part upto the endpoint
        ClosestPolygonPoint from = findClosest(startPoint, boundary_inside[end_inside_poly]);
        ClosestPolygonPoint to = findClosest(from.location, boundary_inside[start_inside_poly]);
        walkToNearestSmallestConnection(from, to);
        
        auto startToBorder = [&]()
        { // from start to border of beginning part
            Point& endPoint = from.location;
            PolygonsPart part = assemblePartContaining(start_inside_poly, start_inside_poly, boundary_inside);
            
            LinePolygonsCrossings linePolygonsCrossings(part, startPoint, endPoint);
            if (!linePolygonsCrossings.lineSegmentCollidesWithBoundary())
            {
                //We're not crossing any boundaries. So skip the comb generation.
                if (!addEndpoint && basicCombPaths.size() == 0) //Only skip if we didn't move the start and end point.
                {
                    combPaths.emplace_back();
                    combPaths.back().push_back(endPoint);
                    return;
                }
                
            }
            linePolygonsCrossings.calcScanlineCrossings();
            
            linePolygonsCrossings.getBasicCombingPathPart(endPoint, basicCombPaths.back());
            
            combPaths.emplace_back();
            Polygons offsettedBoundary = boundary.offset(-(offset - 20));
            optimizePath(offsettedBoundary, startPoint, basicCombPaths.back(), combPaths.back());
        };
        startToBorder();
        
        auto throughAir = [&]()
        { // from start to border of beginning part
            Polygons boundary_outside = boundary.offset(offset);
            Point& startPoint = from.location;
            Point& endPoint = to.location;
            
            LinePolygonsCrossings linePolygonsCrossings(boundary_outside, startPoint, endPoint);
            if (!linePolygonsCrossings.lineSegmentCollidesWithBoundary())
            {
                //We're not crossing any boundaries. So skip the comb generation.
                if (!addEndpoint && basicCombPaths.size() == 0) //Only skip if we didn't move the start and end point.
                {
                    combPaths.emplace_back();
                    combPaths.back().push_back(endPoint);
                    return;
                }
                
            }
            linePolygonsCrossings.calcScanlineCrossings();
            
            linePolygonsCrossings.getBasicCombingPathPart(endPoint, basicCombPaths.back());
            
            combPaths.emplace_back();
            Polygons offsettedBoundary = boundary.offset(-(offset - 20));
            optimizePath(offsettedBoundary, startPoint, basicCombPaths.back(), combPaths.back());
        };
        throughAir();
        
        auto borderToEnd = [&]()
        { // from start to border of beginning part
            Point& startPoint = to.location;
            PolygonsPart part = assemblePartContaining(end_inside_poly, end_inside_poly, boundary_inside);
            
            LinePolygonsCrossings linePolygonsCrossings(part, startPoint, endPoint);
            if (!linePolygonsCrossings.lineSegmentCollidesWithBoundary())
            {
                //We're not crossing any boundaries. So skip the comb generation.
                if (!addEndpoint && basicCombPaths.size() == 0) //Only skip if we didn't move the start and end point.
                {
                    combPaths.emplace_back();
                    combPaths.back().push_back(endPoint);
                    return;
                }
                
            }
            linePolygonsCrossings.calcScanlineCrossings();
            
            linePolygonsCrossings.getBasicCombingPathPart(endPoint, basicCombPaths.back());
            
            combPaths.emplace_back();
            Polygons offsettedBoundary = boundary.offset(-(offset - 20));
            optimizePath(offsettedBoundary, startPoint, basicCombPaths.back(), combPaths.back());
        };
        borderToEnd();
    }
    /*
    
    parts_inside = boundary_inside.splitIntoParts();
    
    //Calculate the minimum and maximum positions where we cross the comb boundary
    calcScanlineCrossings(parts_inside, crossings_inside);
    
    if (min_crossing_dir.part_idx == max_crossing_dir.part_idx)
    {
        getBasicCombingPath(min_crossing_dir.part_idx, crossings_inside[min_crossing_dir.part_idx][min_crossing_dir.crossing_idx], basicCombPaths.back());
    }
    else 
    {
        getBasicCombingPath(min_crossing_dir.part_idx, crossings_inside[min_crossing_dir.part_idx][min_crossing_dir.crossing_idx], basicCombPaths.back());
        
        calcScanlineCrossings(parts_outside, crossings_outside);
        basicCombPaths.emplace_back();
        get path outside 
        
        basicCombPaths.emplace_back();
        getBasicCombingPath(max_crossing_dir.part_idx, crossings_inside[max_crossing_dir.part_idx][max_crossing_dir.crossing_idx], basicCombPaths.back());
        
    }
    getBasicCombingPaths(endPoint, basicCombPaths);
    
    combPaths = basicCombPaths;
    
    */
    
//     bool succeeded = optimizePaths(startPoint, basicCombPaths, combPaths);
//     if (addEndpoint)
//         combPaths.back().push_back(endPoint);
    
//     std::cerr << "succeeded = " << succeeded << std::endl;
//     return succeeded;
    return true;
}

void LinePolygonsCrossings::calcScanlineCrossings()
{
    
    min_crossing_idx = NO_INDEX;
    max_crossing_idx = NO_INDEX;
        
    for(unsigned int poly_idx = 0; poly_idx < boundary.size(); poly_idx++)
    {
        PolyCrossings minMax(poly_idx); 
        PolygonRef poly = boundary[poly_idx];
        Point p0 = transformation_matrix.apply(poly.back());
        for(unsigned int point_idx = 0; point_idx < poly.size(); point_idx++)
        {
            Point p1 = transformation_matrix.apply(poly[point_idx]);
            if ((p0.Y > transformed_startPoint.Y && p1.Y < transformed_startPoint.Y) || (p1.Y > transformed_startPoint.Y && p0.Y < transformed_startPoint.Y))
            {
                int64_t x = p0.X + (p1.X - p0.X) * (transformed_startPoint.Y - p0.Y) / (p1.Y - p0.Y);
                
                if (x >= transformed_startPoint.X && x <= transformed_endPoint.X)
                {
                    if (x < minMax.min.x) { minMax.min.x = x; minMax.min.point_idx = point_idx; }
                    if (x > minMax.max.x) { minMax.max.x = x; minMax.max.point_idx = point_idx; }
                }
            }
            p0 = p1;
        }
        
        if (minMax.min.point_idx != NO_INDEX)
        { // then also max.point_idx != -1
            if (min_crossing_idx == NO_INDEX || minMax.min.x < crossings[min_crossing_idx].min.x) { min_crossing_idx = crossings.size(); }
            if (max_crossing_idx == NO_INDEX || minMax.max.x > crossings[max_crossing_idx].max.x) { max_crossing_idx = crossings.size(); }
            crossings.push_back(minMax);
        }
        
    }
}
/*
void Comb::calcScanlineCrossings(PolygonsPart& part, unsigned int part_idx, PartCrossings& partCrossings)
{
    for(unsigned int poly_idx = 0; poly_idx < part.size(); poly_idx++)
    {
        PolyCrossings minMax(boundary, part_idx, poly_idx); 
        PolygonRef poly = part[poly_idx];
        Point p0 = transformation_matrix.apply(poly.back());
        for(unsigned int point_idx = 0; point_idx < poly.size(); point_idx++)
        {
            Point p1 = transformation_matrix.apply(poly[point_idx]);
            if ((p0.Y > transformed_startPoint.Y && p1.Y < transformed_startPoint.Y) || (p1.Y > transformed_startPoint.Y && p0.Y < transformed_startPoint.Y))
            {
                int64_t x = p0.X + (p1.X - p0.X) * (transformed_startPoint.Y - p0.Y) / (p1.Y - p0.Y);
                
                if (x >= transformed_startPoint.X && x <= transformed_endPoint.X)
                {
                    if (x < minMax.min.x) { minMax.min.x = x; minMax.min.point_idx = point_idx; }
                    if (x > minMax.max.x) { minMax.max.x = x; minMax.max.point_idx = point_idx; }
                    if (x < crossings_inside[min_crossing_dir.part_idx][min_crossing_dir.crossing_idx].x) { min_crossing_dir.part_idx = part_idx; min_crossing_dir.crossing_idx = crossings_inside.size(); }
                    if (x > crossings_inside[max_crossing_dir.part_idx][max_crossing_dir.crossing_idx].x) { max_crossing_dir.part_idx = part_idx; max_crossing_dir.crossing_idx = crossings_inside.size(); }
                }
            }
            p0 = p1;
        }
        if (minMax.min.point_idx != NO_INDEX)
        { // then also max.point_idx != -1
            partCrossings.push_back(minMax);
        }
    }
}*/

void LinePolygonsCrossings::getBasicCombingPathPart(Point endPoint, CombPath& combPath) 
{
    for (PolyCrossings crossing = getNextPolygonAlongScanline(transformed_startPoint.X)
        ; crossing.poly_idx != NO_INDEX
        ; crossing = getNextPolygonAlongScanline(crossing.max.x))
    {
        getBasicCombingPath(crossing, combPath);
    }
    combPath.push_back(endPoint);
}

void LinePolygonsCrossings::getBasicCombingPath(PolyCrossings polyCrossings, std::vector<Point>& pointList) 
{
    PolygonRef poly = boundary[polyCrossings.poly_idx];
    pointList.push_back(transformation_matrix.unapply(Point(polyCrossings.min.x, transformed_startPoint.Y)));
    if ( ( polyCrossings.max.point_idx - polyCrossings.min.point_idx + poly.size() ) % poly.size() 
        < poly.size() / 2 )
    { // follow the path in the same direction as the winding order of the boundary polygon
        for(unsigned int point_idx = polyCrossings.min.point_idx
            ; point_idx != polyCrossings.max.point_idx
            ; point_idx = (point_idx < poly.size() - 1) ? (point_idx + 1) : (0))
        {
            pointList.push_back(poly[point_idx]);
        }
    }
    else
    {
        unsigned int min_idx = (polyCrossings.min.point_idx == 0)? poly.size() - 1: polyCrossings.min.point_idx - 1;
        unsigned int max_idx = (polyCrossings.max.point_idx == 0)? poly.size() - 1: polyCrossings.max.point_idx - 1;
        
        for(unsigned int point_idx = min_idx; point_idx != max_idx; point_idx = (point_idx > 0) ? (point_idx - 1) : (poly.size() - 1))
        {
            pointList.push_back(poly[point_idx]);
        }
    }
    pointList.push_back(transformation_matrix.unapply(Point(polyCrossings.max.x, transformed_startPoint.Y))); 
}



LinePolygonsCrossings::PolyCrossings LinePolygonsCrossings::getNextPolygonAlongScanline(int64_t x)
{
    PolyCrossings ret(NO_INDEX);
    for(PolyCrossings& crossing : crossings)
    {
        if (crossing.min.x > x && crossing.min.x < ret.min.x)
        {
            ret = crossing;
        }
    }
    return ret;
}



bool Comb::optimizePath(Polygons& offsettedBoundary, Point startPoint, std::vector<Point>& pointList, CombPath& combPoints) 
{
//     for (Point& p : pointList)
//         combPoints.push_back(p);
    
    Point current_point = startPoint;
    for(unsigned int point_idx = 1; point_idx<pointList.size(); point_idx++)
    {
        if (polygonCollidesWithlineSegment(offsettedBoundary, current_point, pointList[point_idx]))
        {
            if (polygonCollidesWithlineSegment(offsettedBoundary, current_point, pointList[point_idx - 1]))
            {
                return false;
            }
            current_point = pointList[point_idx - 1];
            combPoints.push_back(current_point);
        }
        else 
        {
            //dont add the newest point
            while (combPoints.size() > 0)
            {
                if (polygonCollidesWithlineSegment(offsettedBoundary, combPoints.back(), pointList[point_idx]))
                {
                    break;
                }
                else 
                {
                    combPoints.pop_back();
                }
            }
            if (combPoints.size() > 0)
            {
                current_point = combPoints.back();
            }
        }
    }
    return true;
}














/*


void Comb::calcScanlineCrossings()
{
    calcScanlineCrossings(parts_inside, crossings_inside);
    calcScanlineCrossings(parts_outside, crossings_outside);
}
void Comb::calcScanlineCrossings(std::vector<PolygonsPart>& boundary, PartCrossings* partCrossingsArray)
{
    for(unsigned int part_idx = 0; part_idx < boundary.size(); part_idx++)
    {
        PolygonsPart& part = boundary[part_idx];
        calcScanlineCrossings(part, part_idx, partCrossingsArray[part_idx]);
    }
}


PolyCrossings Comb::getNextPolygonAlongScanline(PartCrossings crossings, int64_t x)
{
    PolyCrossings ret(parts_inside, NO_INDEX, NO_INDEX);
    for(PolyCrossings& crossing : crossings)
    {
        if (crossing.min.x > x && crossing.min.x < ret.min.x)
        {
            ret = crossing;
        }
    }
    return ret;
}

PolyCrossings Comb::getNextPolygonAlongScanline(bool inside, int64_t x)
{
    PolyCrossings ret(parts_inside, NO_INDEX, NO_INDEX);
    for (unsigned int part_idx = 0; part_idx < (inside)? parts_inside.size() : parts_outside.size(); part_idx++)
    {
        PartCrossings& crossings = (inside)? crossings_inside[part_idx] : crossings_outside[part_idx];
        for(PolyCrossings& crossing : crossings)
        {
            if (crossing.min.x > x && crossing.min.x < ret.min.x)
            {
                ret = crossing;
            }
        }
    }
    return ret;
}




void Comb::getBasicCombingPathPart(Point endPoint, CombPath& combPath) 
{
    unsigned int part_idx = min_crossing_dir.part_idx;
    combPath.part_idx = part_idx;
//     PartCrossings& min_crossing = crossings_inside[min_crossing_dir.part_idx][min_crossing_dir.crossing_idx];
    for (PolyCrossings crossing = getNextPolygonAlongScanline(crossings_inside[part_idx], transformed_startPoint.X)
        ; crossing.poly_idx != NO_INDEX
        ; crossing = getNextPolygonAlongScanline(crossing.max))
    {
        getBasicCombingPath(part_idx, crossing, combPath);
    }
    combPath.push_back(endPoint);
}

void Comb::getBasicCombingPath(unsigned int part_idx, PolyCrossings crossings, std::vector<Point>& pointList) 
{
    PolygonsPart& part = parts_inside[part_idx];
    PolygonRef poly = part[crossings.poly_idx];
    pointList.push_back(transformation_matrix.unapply(Point(crossings.min.x - MM2INT(0.2), transformed_startPoint.Y)));
    if ( ( crossings.max.point_idx - crossings.min.point_idx + poly.size() ) % poly.size() 
        < poly.size() / 2 )
    { // follow the path in the same direction as the winding order of the boundary polygon
        for(unsigned int point_idx = crossings.min.point_idx
            ; point_idx != crossings.max.point_idx
            ; point_idx = (point_idx < part.size() - 1) ? (point_idx + 1) : (0))
        {
            pointList.push_back(poly[point_idx]);
        }
    }
    else
    {
        unsigned int min_idx = (crossings.min.point_idx == 0)? poly.size() - 1: crossings.min.point_idx - 1;
        unsigned int max_idx = (crossings.max.point_idx == 0)? poly.size() - 1: crossings.max.point_idx - 1;
        
        for(unsigned int point_idx = min_idx; point_idx != max_idx; point_idx = (point_idx > 0) ? (point_idx - 1) : (poly.size() - 1))
        {
            pointList.push_back(poly[point_idx]);
        }
    }
    pointList.push_back(transformation_matrix.unapply(Point(crossings.max.x + MM2INT(0.2), transformed_startPoint.Y))); //TODO: hard coded unnamed value!
}

bool Comb::optimizePath(Point startPoint, std::vector<Point>& pointList, std::vector<Point>& combPoints) 
{
    
    Point current_point = startPoint;
    for(unsigned int point_idx = 1; point_idx<pointList.size(); point_idx++)
    {
        if (lineSegmentCollidesWithAnyInsideBoundary(current_point, pointList[point_idx]))
        {
            if (lineSegmentCollidesWithAnyInsideBoundary(current_point, pointList[point_idx - 1]))
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
}*/

}//namespace cura
