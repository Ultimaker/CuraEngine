/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "comb.h"

#include <algorithm>

#include "debug.h"
#include "utils/polygonUtils.h"

namespace cura {

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

unsigned int Comb::moveInside_(Point& from, int distance)
{
    return moveInside(boundary_inside, from, distance, max_moveInside_distance2);
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
    PartsView partsView = boundary_inside.splitIntoPartsView(); // !! changed the order of boundary_inside 
   
      

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
        start_inside_poly = moveInside(boundary_inside, startPoint, 100, max_moveInside_distance2);
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
        end_inside_poly = moveInside(boundary_inside, endPoint, 100, max_moveInside_distance2);
        if (end_inside_poly == NO_INDEX)    //If we fail to move the point inside the comb boundary we need to retract.
        {
            std::cerr << " fail to move the end point inside the comb boundary we need to retract. <<<<<<<<<<<< "<< std::endl;
            return false;
        }
        addEndpoint = true;
    }

    
//     std::cerr << "calculating comb path!" << std::endl;
    DEBUG_SHOW(boundary_inside.size());
    DEBUG_SHOW(start_inside_poly);
    DEBUG_SHOW(end_inside_poly);
    
    int64_t offset_dist_to_get_from_on_the_polygon_to_outside = 20;
    
    auto comb = [](Polygons& relevantPolygons, Polygons& relevantPolygonsLessOffset, Point startPoint, Point endPoint, CombPath& combPath)
    {
        
        LinePolygonsCrossings linePolygonsCrossings(relevantPolygons, startPoint, endPoint);
        linePolygonsCrossings.getCombingPath(relevantPolygonsLessOffset, combPath);
    };
    
    unsigned int start_part_boundary_poly_idx;
    unsigned int end_part_boundary_poly_idx;
    unsigned int start_part_idx = partsView.getPartContaining(start_inside_poly, &start_part_boundary_poly_idx);
    unsigned int end_part_idx = partsView.getPartContaining(end_inside_poly, &end_part_boundary_poly_idx);
    
    if (start_part_idx == end_part_idx)
    { // normal combing within part
        PolygonsPart part = partsView.assemblePart(start_part_idx);
        Polygons part_extra_offset = part.offset(offset_dist_to_get_from_on_the_polygon_to_outside);
        combPaths.emplace_back();
        comb(part, part_extra_offset, startPoint, endPoint, combPaths.back());
        return true;
    }
    else 
    { // comb to edge >> move through air avoiding other parts >> comb inside end part upto the endpoint
//         return false;
        DEBUG_PRINTLN(">>>>>>>>>>>>>> combing via outside!! " );
        DEBUG_SHOW(start_part_idx);
        DEBUG_SHOW(end_part_idx);
        ClosestPolygonPoint from = findClosest(endPoint, boundary_inside[start_part_boundary_poly_idx]);
        ClosestPolygonPoint to = findClosest(from.location, boundary_inside[end_part_boundary_poly_idx]);
        walkToNearestSmallestConnection(from, to);
        
        DEBUG_PRINTLN("start to boundary");
        // start to boundary
        PolygonsPart part_begin = partsView.assemblePart(start_part_idx);
        Polygons part_begin_extra_offset = part_begin.offset(offset_dist_to_get_from_on_the_polygon_to_outside);
        combPaths.emplace_back();
        comb(part_begin, part_begin_extra_offset, startPoint, from.location, combPaths.back());
        
        DEBUG_PRINTLN("throught air from boundary to boundary");
         // throught air from boundary to boundary
        Polygons middle = boundary.offset(offset);
        Polygons middle_extra_offset = middle.offset(-offset_dist_to_get_from_on_the_polygon_to_outside);
        combPaths.emplace_back();
        
        Point from_outside = from.location;
        moveInside(middle, from_outside, -100, max_moveInside_distance2);
        
        Point to_outside = to.location;
        moveInside(middle, to_outside, -100, max_moveInside_distance2);
        /*
        combPaths.back().push_back(from.location);
        combPaths.back().push_back(Point(from.location.X, 0));
        combPaths.back().push_back(Point(from_outside.X, 0));
        combPaths.back().push_back(from_outside);
        combPaths.back().push_back(Point(from_outside.X, 0));
        combPaths.back().push_back(Point(to_outside.X, 0));
        combPaths.back().push_back(to_outside);
        */
        comb(middle, middle_extra_offset, from_outside, to_outside, combPaths.back());
        
        DEBUG_PRINTLN(" boundary to end");
         // boundary to end
        PolygonsPart part_end = partsView.assemblePart(end_part_idx);
        Polygons part_end_extra_offset = part_end.offset(offset_dist_to_get_from_on_the_polygon_to_outside);
        combPaths.emplace_back();
        comb(part_end, part_end_extra_offset, to.location, endPoint, combPaths.back());
        
        return true;
    }
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


void LinePolygonsCrossings::getCombingPath(Polygons& offsettedBoundary, CombPath& combPath)
{
    if (!lineSegmentCollidesWithBoundary())
    {
        //We're not crossing any boundaries. So skip the comb generation.
        DEBUG_PRINTLN("combing can go straight without problem! <<<<<<<<<<<< ");
        combPath.push_back(startPoint); // TODO: do this?
        combPath.push_back(endPoint); // TODO: do this?
        return; // true
    }
    calcScanlineCrossings();
    
    CombPath basicPath;
    getBasicCombingPathPart(endPoint, basicPath);
    optimizePath(offsettedBoundary, startPoint, basicPath, combPath);
}


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


bool LinePolygonsCrossings::optimizePath(Polygons& offsettedBoundary, Point startPoint, std::vector<Point>& pointList, CombPath& combPoints) 
{
    for (Point& p : pointList)
        combPoints.push_back(p);
    /*
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
    return true;*/
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
