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

Polygons Comb::getLayerOuterWalls(SliceDataStorage& storage, unsigned int layer_nr)
{
    Polygons layer_walls;
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        for (SliceLayerPart& part : mesh.layers[layer_nr].parts)
        {
            if (part.insets.size() > 0)
            {
                layer_walls.add(part.insets[0]);
            }
            else 
            {
                layer_walls.add(part.outline.offset(-offset_from_outlines));
            }
        }
    }
    return layer_walls;
}
    
Comb::Comb(SliceDataStorage& storage, unsigned int layer_nr)
: boundary( getLayerOutlines(storage, layer_nr) )
, boundary_inside( getLayerOuterWalls(storage, layer_nr) )
// , boundary_inside( boundary.offset(-offset_from_outlines) )
, partsView_inside( boundary_inside.splitIntoPartsView() ) // !! changes the order of boundary_inside 
, boundary_outside(nullptr)
, boundary_outside_extra_offset(nullptr)
{
}

Comb::~Comb()
{
    if (boundary_outside)
        delete boundary_outside;
    if (boundary_outside_extra_offset)
        delete boundary_outside_extra_offset;
}

unsigned int Comb::moveInside_(Point& from, int distance)
{
    return moveInside(boundary_inside, from, distance, max_moveInside_distance2);
}

Polygons* Comb::getBoundaryOutside()
{
    if (!boundary_outside)
    {
        boundary_outside = new Polygons();
        *boundary_outside = boundary.offset(offset_from_outlines_outside);
    }
    return boundary_outside;
}
Polygons* Comb::getBoundaryOutsideExtraOffset()
{
    if (!boundary_outside_extra_offset)
    {
        boundary_outside_extra_offset = new Polygons();
        *boundary_outside_extra_offset = getBoundaryOutside()->offset(-Comb::offset_dist_to_get_from_on_the_polygon_to_outside);
    }
    return boundary_outside_extra_offset;
}

bool Comb::calc(Point startPoint, Point endPoint, CombPaths& combPaths)
{
    if (shorterThen(endPoint - startPoint, max_comb_distance_ignored))
    {
        return true;
    }
    
//     DEBUG_PRINTLN(" >>>>>>>>>>>>>>>>>>>>>> ");
    
    
    bool startInside = true;
    bool endInside = true;
    
    //Move start and end point inside the comb boundary
    unsigned int start_inside_poly = boundary_inside.findInside(startPoint, true);
    if (start_inside_poly == NO_INDEX)
    {
        start_inside_poly = moveInside(boundary_inside, startPoint, offset_extra_start_end, max_moveInside_distance2);
        if (start_inside_poly == NO_INDEX)    //If we fail to move the point inside the comb boundary we need to retract.
        {   
            startInside = false;
        }
    }
    unsigned int end_inside_poly = boundary_inside.findInside(endPoint, true);
    if (end_inside_poly == NO_INDEX)
    {
        end_inside_poly = moveInside(boundary_inside, endPoint, offset_extra_start_end, max_moveInside_distance2);
        if (end_inside_poly == NO_INDEX)    //If we fail to move the point inside the comb boundary we need to retract.
        {
            endInside = false;
        }
    }
    

    
    unsigned int start_part_boundary_poly_idx;
    unsigned int end_part_boundary_poly_idx;
    unsigned int start_part_idx = partsView_inside.getPartContaining(start_inside_poly, &start_part_boundary_poly_idx);
    unsigned int end_part_idx = partsView_inside.getPartContaining(end_inside_poly, &end_part_boundary_poly_idx);
    
    if (startInside && endInside && start_part_idx == end_part_idx)
    { // normal combing within part
        PolygonsPart part = partsView_inside.assemblePart(start_part_idx);
        Polygons part_extra_offset = part; // .offset(Comb::offset_dist_to_get_from_on_the_polygon_to_outside); TODO
        combPaths.emplace_back();
        LinePolygonsCrossings::comb(part, part_extra_offset, startPoint, endPoint, combPaths.back());
        return true;
    }
    else 
        return false;
//     if (false)
    { // comb inside part to edge (if needed) >> move through air avoiding other parts >> comb inside end part upto the endpoint (if needed) 
        Point middle_from;
        Point middle_to;
        
        if (startInside && endInside)
        {
            ClosestPolygonPoint middle_from_cp = findClosest(endPoint, boundary_inside[start_part_boundary_poly_idx]);
            ClosestPolygonPoint middle_to_cp = findClosest(middle_from_cp.location, boundary_inside[end_part_boundary_poly_idx]);
//             walkToNearestSmallestConnection(middle_from_cp, middle_to_cp); // TODO: remove this line???
            middle_from = middle_from_cp.location;
            middle_to = middle_to_cp.location;
        }
        else 
        {
            if (!startInside && !endInside) 
            { 
                middle_from = startPoint; 
                middle_to = endPoint; 
            }
            else if (!startInside && endInside)
            {
                middle_from = startPoint;
                ClosestPolygonPoint middle_to_cp = findClosest(middle_from, boundary_inside[end_part_boundary_poly_idx]);
                middle_to = middle_to_cp.location;
            }
            else if (startInside && !endInside)
            {
                middle_to = endPoint;
                ClosestPolygonPoint middle_from_cp = findClosest(middle_to, boundary_inside[start_part_boundary_poly_idx]);
                middle_from = middle_from_cp.location;
            }
        }
        
        if (startInside)
        {
            // start to boundary
            PolygonsPart part_begin = partsView_inside.assemblePart(start_part_idx); // comb through the starting part only
            Polygons part_begin_extra_offset = part_begin; // .offset(Comb::offset_dist_to_get_from_on_the_polygon_to_outside);  TODO
            combPaths.emplace_back();
            LinePolygonsCrossings::comb(part_begin, part_begin_extra_offset, startPoint, middle_from, combPaths.back());
        }
        
         // throught air from boundary to boundary
        Polygons& middle = *getBoundaryOutside(); // comb through all air, since generally the outside consists of a single part
        Polygons& middle_extra_offset = middle; // *getBoundaryOutsideExtraOffset();  TODO
        Point from_outside = middle_from;
        if (startInside || middle.findInside(from_outside, true) == NO_INDEX)
        {
            moveInside(middle, from_outside, -offset_extra_start_end, max_moveInside_distance2);
        }
        Point to_outside = middle_to;
        if (endInside || middle.findInside(to_outside, true) == NO_INDEX)
        {
            moveInside(middle, to_outside, -offset_extra_start_end, max_moveInside_distance2);
        }
        combPaths.emplace_back();
        combPaths.back().throughAir = true;
        LinePolygonsCrossings::comb(middle, middle_extra_offset, from_outside, to_outside, combPaths.back());
        
        if (endInside)
        {
            // boundary to end
            PolygonsPart part_end = partsView_inside.assemblePart(end_part_idx); // comb through end part only
            Polygons part_end_extra_offset = part_end; //.offset(Comb::offset_dist_to_get_from_on_the_polygon_to_outside);  TODO
            combPaths.emplace_back();
            LinePolygonsCrossings::comb(part_end, part_end_extra_offset, middle_to, endPoint, combPaths.back());
        }
        
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
//         DEBUG_PRINTLN("combing can go straight without problem! <<<<<<<<<<<< ");
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
    auto getPoint = [&](unsigned int point_idx)
    {
//         return poly[point_idx];
        return getBoundaryPointWithOffset(poly, point_idx, Comb::offset_dist_to_get_from_on_the_polygon_to_outside);
    };
    combPath.push_back(transformation_matrix.unapply(Point(polyCrossings.min.x, transformed_startPoint.Y)));
    if ( ( polyCrossings.max.point_idx - polyCrossings.min.point_idx + poly.size() ) % poly.size() 
        < poly.size() / 2 )
    { // follow the path in the same direction as the winding order of the boundary polygon
        for(unsigned int point_idx = polyCrossings.min.point_idx
            ; point_idx != polyCrossings.max.point_idx
            ; point_idx = (point_idx < poly.size() - 1) ? (point_idx + 1) : (0))
        {
            combPath.push_back(getPoint(point_idx));
        }
    }
    else
    {
        unsigned int min_idx = (polyCrossings.min.point_idx == 0)? poly.size() - 1: polyCrossings.min.point_idx - 1;
        unsigned int max_idx = (polyCrossings.max.point_idx == 0)? poly.size() - 1: polyCrossings.max.point_idx - 1;
        
        for(unsigned int point_idx = min_idx; point_idx != max_idx; point_idx = (point_idx > 0) ? (point_idx - 1) : (poly.size() - 1))
        {
            combPath.push_back(getPoint(point_idx));
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
//     for (Point& p : comb_path)
//         optimized_comb_path.push_back(p);
//     return true;
    
    optimized_comb_path.push_back(startPoint);
    for(unsigned int point_idx = 1; point_idx<comb_path.size(); point_idx++)
    {
        Point& current_point = optimized_comb_path.back();
        if (polygonCollidesWithlineSegment(offsettedBoundary, current_point, comb_path[point_idx]))
        {
//             if (polygonCollidesWithlineSegment(offsettedBoundary, current_point, comb_path[point_idx - 1]))
//             {
//                 return false;
//             }
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
