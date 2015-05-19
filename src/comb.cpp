/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "comb.h"

#include <algorithm>

#include "debug.h"
#include "utils/polygonUtils.h"
#include "sliceDataStorage.h"

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


Polygons Comb::getLayerOutlines(SliceDataStorage& storage, unsigned int layer_nr)
{
        Polygons layer_outlines;
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        for (SliceLayerPart& part : mesh.layers[layer_nr].parts)
        {
            layer_outlines.add(part.outline);
        }
    }
    return layer_outlines;
}
    
Comb::Comb(SliceDataStorage& storage, unsigned int layer_nr)
: boundary( getLayerOutlines(storage, layer_nr) )
, boundary_inside( boundary.offset(-offset_from_outlines) )
, partsView_inside( boundary_inside.splitIntoPartsView() ) // !! changes the order of boundary_inside 
{
}

Comb::~Comb()
{
}

unsigned int Comb::moveInside_(Point& from, int distance)
{
    return moveInside(boundary_inside, from, distance, max_moveInside_distance2);
}

bool Comb::calc(Point startPoint, Point endPoint, CombPaths& combPaths)
{
    if (shorterThen(endPoint - startPoint, max_comb_distance_ignored))
    {
        return true;
    }
    
    DEBUG_PRINTLN(" >>>>>>>>>>>>>>>>>>>>>> ");
    
    CombPaths basicCombPaths;
    basicCombPaths.emplace_back();
    
    
    //Check if we are inside the comb boundaries
    unsigned int start_inside_poly = boundary_inside.findInside(startPoint, true);
    if (start_inside_poly == NO_INDEX)
    {
        start_inside_poly = moveInside(boundary_inside, startPoint, offset_extra_start_end, max_moveInside_distance2);
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
        end_inside_poly = moveInside(boundary_inside, endPoint, offset_extra_start_end, max_moveInside_distance2);
        if (end_inside_poly == NO_INDEX)    //If we fail to move the point inside the comb boundary we need to retract.
        {
            std::cerr << " fail to move the end point inside the comb boundary we need to retract. <<<<<<<<<<<< "<< std::endl;
            return false;
        }
    }
    

    
    unsigned int start_part_boundary_poly_idx;
    unsigned int end_part_boundary_poly_idx;
    unsigned int start_part_idx = partsView_inside.getPartContaining(start_inside_poly, &start_part_boundary_poly_idx);
    unsigned int end_part_idx = partsView_inside.getPartContaining(end_inside_poly, &end_part_boundary_poly_idx);
    
    if (start_part_idx == end_part_idx)
    { // normal combing within part
        PolygonsPart part = partsView_inside.assemblePart(start_part_idx);
        Polygons part_extra_offset = part.offset(Comb::offset_dist_to_get_from_on_the_polygon_to_outside);
        combPaths.emplace_back();
        LinePolygonsCrossings::comb(part, part_extra_offset, startPoint, endPoint, combPaths.back());
        return true;
    }
    else 
    { // comb to edge >> move through air avoiding other parts >> comb inside end part upto the endpoint
        DEBUG_PRINTLN(">>>>>>>>>>>>>> combing via outside!! " );
        DEBUG_SHOW(start_part_idx);
        DEBUG_SHOW(end_part_idx);
        ClosestPolygonPoint from = findClosest(endPoint, boundary_inside[start_part_boundary_poly_idx]);
        ClosestPolygonPoint to = findClosest(from.location, boundary_inside[end_part_boundary_poly_idx]);
        walkToNearestSmallestConnection(from, to);
        
        DEBUG_PRINTLN("start to boundary");
        // start to boundary
        PolygonsPart part_begin = partsView_inside.assemblePart(start_part_idx);
        Polygons part_begin_extra_offset = part_begin.offset(Comb::offset_dist_to_get_from_on_the_polygon_to_outside);
        combPaths.emplace_back();
        LinePolygonsCrossings::comb(part_begin, part_begin_extra_offset, startPoint, from.location, combPaths.back());
        
        DEBUG_PRINTLN("throught air from boundary to boundary");
         // throught air from boundary to boundary
        Polygons middle = boundary.offset(offset_from_outlines);
        Polygons middle_extra_offset = middle.offset(-Comb::offset_dist_to_get_from_on_the_polygon_to_outside);
        Point from_outside = from.location;
        moveInside(middle, from_outside, -offset_extra_start_end, max_moveInside_distance2);
        Point to_outside = to.location;
        moveInside(middle, to_outside, -offset_extra_start_end, max_moveInside_distance2);
        combPaths.emplace_back();
        LinePolygonsCrossings::comb(middle, middle_extra_offset, from_outside, to_outside, combPaths.back());
        
        DEBUG_PRINTLN(" boundary to end");
         // boundary to end
        PolygonsPart part_end = partsView_inside.assemblePart(end_part_idx);
        Polygons part_end_extra_offset = part_end.offset(Comb::offset_dist_to_get_from_on_the_polygon_to_outside);
        combPaths.emplace_back();
        LinePolygonsCrossings::comb(part_end, part_end_extra_offset, to.location, endPoint, combPaths.back());
        
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

void LinePolygonsCrossings::getCombingPath(Polygons& offsettedBoundary, CombPath& combPath)
{
    if (shorterThen(endPoint - startPoint, Comb::max_comb_distance_ignored) || !lineSegmentCollidesWithBoundary())
    {
        //We're not crossing any boundaries. So skip the comb generation.
        DEBUG_PRINTLN("combing can go straight without problem! <<<<<<<<<<<< ");
        combPath.push_back(startPoint); // TODO: do this?
        combPath.push_back(endPoint); // TODO: do this?
        return; // true
    }
    calcScanlineCrossings();
    
    CombPath basicPath;
    getBasicCombingPath(basicPath);
    optimizePath(offsettedBoundary, basicPath, combPath);
}


void LinePolygonsCrossings::getBasicCombingPath(CombPath& combPath) 
{
    for (PolyCrossings crossing = getNextPolygonAlongScanline(transformed_startPoint.X)
        ; crossing.poly_idx != NO_INDEX
        ; crossing = getNextPolygonAlongScanline(crossing.max.x))
    {
        getBasicCombingPath(crossing, combPath);
    }
    combPath.push_back(endPoint);
}

void LinePolygonsCrossings::getBasicCombingPath(PolyCrossings polyCrossings, CombPath& combPath) 
{
    PolygonRef poly = boundary[polyCrossings.poly_idx];
    combPath.push_back(transformation_matrix.unapply(Point(polyCrossings.min.x, transformed_startPoint.Y)));
    if ( ( polyCrossings.max.point_idx - polyCrossings.min.point_idx + poly.size() ) % poly.size() 
        < poly.size() / 2 )
    { // follow the path in the same direction as the winding order of the boundary polygon
        for(unsigned int point_idx = polyCrossings.min.point_idx
            ; point_idx != polyCrossings.max.point_idx
            ; point_idx = (point_idx < poly.size() - 1) ? (point_idx + 1) : (0))
        {
            combPath.push_back(poly[point_idx]);
        }
    }
    else
    {
        unsigned int min_idx = (polyCrossings.min.point_idx == 0)? poly.size() - 1: polyCrossings.min.point_idx - 1;
        unsigned int max_idx = (polyCrossings.max.point_idx == 0)? poly.size() - 1: polyCrossings.max.point_idx - 1;
        
        for(unsigned int point_idx = min_idx; point_idx != max_idx; point_idx = (point_idx > 0) ? (point_idx - 1) : (poly.size() - 1))
        {
            combPath.push_back(poly[point_idx]);
        }
    }
    combPath.push_back(transformation_matrix.unapply(Point(polyCrossings.max.x, transformed_startPoint.Y))); 
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
bool LinePolygonsCrossings::optimizePath(Polygons& offsettedBoundary, CombPath& comb_path, CombPath& optimized_comb_path) 
{
    optimized_comb_path.push_back(startPoint);
    for(unsigned int point_idx = 1; point_idx<comb_path.size(); point_idx++)
    {
        Point& current_point = optimized_comb_path.back();
        if (polygonCollidesWithlineSegment(offsettedBoundary, current_point, comb_path[point_idx]))
        {
            if (polygonCollidesWithlineSegment(offsettedBoundary, current_point, comb_path[point_idx - 1]))
            {
                return false;
            }
            optimized_comb_path.push_back(comb_path[point_idx - 1]);
        }
        else 
        {
            //dont add the newest point
            while (optimized_comb_path.size() > 1)
            {
                if (polygonCollidesWithlineSegment(offsettedBoundary, optimized_comb_path[optimized_comb_path.size() - 2], comb_path[point_idx]))
                {
                    break;
                }
                else 
                {
                    optimized_comb_path.pop_back();
                }
            }
        }
    }
    return true;
}

}//namespace cura
