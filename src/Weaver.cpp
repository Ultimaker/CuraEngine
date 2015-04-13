#include "Weaver.h"

#include <cmath> // sqrt
#include <fstream> // debug IO
#include <unistd.h>

#include "weaveDataStorage.h"

using namespace cura;


/*!
 * This is the main function for Neith / Weaving / WirePrinting / Webbed printing.
 * Creates a wireframe for the model consisting of horizontal 'flat' parts and connections between consecutive flat parts consisting of UP moves and diagonally DOWN moves.
 */
void Weaver::weave(PrintObject* object, CommandSocket* commandSocket)
{
    int maxz = object->max().z;

    int layer_count = (maxz - initial_layer_thickness) / connectionHeight + 1;

    DEBUG_SHOW(layer_count);

    std::vector<cura::Slicer*> slicerList;

    for(Mesh& mesh : object->meshes)
    {
        int fix_horrible = mesh.getSettingInt("fixHorrible");
        cura::Slicer* slicer = new cura::Slicer(&mesh, initial_layer_thickness, connectionHeight, layer_count, fix_horrible & FIX_HORRIBLE_KEEP_NONE_CLOSED, fix_horrible & FIX_HORRIBLE_EXTENSIVE_STITCHING);
        slicerList.push_back(slicer);
    }

    
    int starting_layer_idx;
    { // find first non-empty layer
        for (starting_layer_idx = 0; starting_layer_idx < layer_count; starting_layer_idx++)
        {
            Polygons parts;
            for (cura::Slicer* slicer : slicerList)
                parts.add(slicer->layers[starting_layer_idx].polygonList);  
            
            if (parts.size() > 0)
                break;
        }
        if (starting_layer_idx > 0)
        {
            logError("First %i layers are empty!\n", starting_layer_idx);
        }
    }
    
    
    std::cerr<< "chainifying layers..." << std::endl;
    {
        int starting_z = -1;
        for (cura::Slicer* slicer : slicerList)
            wireFrame.bottom_outline.add(slicer->layers[starting_layer_idx].polygonList);
        
        if (commandSocket)
            commandSocket->sendPolygons(Inset0Type, 0, wireFrame.bottom_outline);
        
        wireFrame.z_bottom = slicerList[0]->layers[starting_layer_idx].z;
        
        Point starting_point_in_layer;
        if (wireFrame.bottom_outline.size() > 0)
            starting_point_in_layer = (wireFrame.bottom_outline.max() + wireFrame.bottom_outline.min()) / 2;
        else 
            starting_point_in_layer = (Point(0,0) + object->max() + object->min()) / 2;
        
        for (int layer_idx = starting_layer_idx + 1; layer_idx < layer_count; layer_idx++)
        {
            logProgress("inset", layer_idx+1, layer_count); // abuse the progress system of the normal mode of CuraEngine
            
            Polygons parts1;
            for (cura::Slicer* slicer : slicerList)
                parts1.add(slicer->layers[layer_idx].polygonList);

            
            Polygons chainified;

            chainify_polygons(parts1, starting_point_in_layer, chainified, false);
            
            if (commandSocket)
                commandSocket->sendPolygons(Inset0Type, layer_idx - starting_layer_idx, chainified);
            
            if (chainified.size() > 0)
            {
                if (starting_z == -1) starting_z = slicerList[0]->layers[layer_idx-1].z;
                wireFrame.layers.emplace_back();
                WeaveLayer& layer = wireFrame.layers.back();
                
                layer.z0 = slicerList[0]->layers[layer_idx-1].z - starting_z;
                layer.z1 = slicerList[0]->layers[layer_idx].z - starting_z;
                
                layer.supported = chainified;
                
                starting_point_in_layer = layer.supported.back().back();
            }
        }
    }
    
    std::cerr<< "finding horizontal parts..." << std::endl;
    {
        Polygons* lower_top_parts = &wireFrame.bottom_outline;
        
        for (int layer_idx = 0; layer_idx < wireFrame.layers.size(); layer_idx++)
        {
            logProgress("skin", layer_idx+1, wireFrame.layers.size()); // abuse the progress system of the normal mode of CuraEngine
            
            WeaveLayer& layer = wireFrame.layers[layer_idx];
            
            Polygons empty;
            Polygons& layer_above = (layer_idx+1 < wireFrame.layers.size())? wireFrame.layers[layer_idx+1].supported : empty;
            
            createHorizontalFill(*lower_top_parts, layer, layer_above, layer.z1);
            lower_top_parts = &layer.supported;
        }
    }
    // at this point layer.supported still only contains the polygons to be connected
    // when connecting layers, we further add the supporting polygons created by the roofs
    
    std::cerr<< "connecting layers..." << std::endl;
    {
        Polygons* lower_top_parts = &wireFrame.bottom_outline;
        int last_z = wireFrame.z_bottom;
        for (int layer_idx = 0; layer_idx < wireFrame.layers.size(); layer_idx++) // use top of every layer but the last
        {
            WeaveLayer& layer = wireFrame.layers[layer_idx];
            
            connect_polygons(*lower_top_parts, last_z, layer.supported, layer.z1, layer);
            layer.supported.add(layer.roofs.roof_outlines);
            lower_top_parts = &layer.supported;
            
            last_z = layer.z1;
        }
    }


    { // roofs:
        
        WeaveLayer& top_layer = wireFrame.layers.back();
        Polygons to_be_supported; // empty for the top layer
        fillRoofs(top_layer.supported, to_be_supported, -1, top_layer.z1, top_layer.roofs);
    }
    
    
    { // bottom:
        Polygons to_be_supported; // is empty for the bottom layer, cause the order of insets doesn't really matter (in a sense everything is to be supported)
        fillRoofs(wireFrame.bottom_outline, to_be_supported, -1, wireFrame.layers.front().z0, wireFrame.bottom_infill);
    }
    
}


/*!
 * Creates the roofs and floors which are laid down horizontally.
 */
void Weaver::createHorizontalFill(Polygons& lower_top_parts, WeaveLayer& layer, Polygons& layer_above, int z1)
{
    int64_t bridgable_dist = connectionHeight;
    
    Polygons& polys_below = lower_top_parts;
    Polygons& polys_here = layer.supported;
    Polygons& polys_above = layer_above;

    
    { // roofs
        Polygons to_be_supported =  polys_above.offset(bridgable_dist);
        fillRoofs(polys_here, to_be_supported, -1, layer.z1, layer.roofs);

    }
   
    { // floors
        Polygons to_be_supported =  polys_above.offset(-bridgable_dist);
        fillFloors(polys_here, to_be_supported, 1, layer.z1, layer.roofs);
    }
    
    {// optimize away doubly printed regions (boundaries of holes in layer etc.)
        for (WeaveRoofPart& inset : layer.roofs.roof_insets)
            connections2moves(inset);
    }
    
}
    
/*!
 * Fill roofs starting from the outlines of \p supporting.
 * The area to be filled in is difference( \p to_be_supported , \p supporting ).
 * 
 * The basic algorithm performs insets on \p supported until the whole area of \p to_be_supported is filled.
 * In order to not fill holes in the roof, the hole-areas are unioned with the insets, which results in connections where the UP move has close to zero length;
 * pieces of the area between two consecutive insets have close to zero distance at these points.
 * These parts of the horizontal infills are converted into moves by the function \p connections2moves.
 * 
 * Note that the new inset is computed from the last inset, while the connections are between the last chainified inset and the new chainified inset.
 * 
 */
void Weaver::fillRoofs(Polygons& supporting, Polygons& to_be_supported, int direction, int z, WeaveRoof& horizontals)
{
    std::vector<WeaveRoofPart>& insets = horizontals.roof_insets;
    
    if (supporting.size() == 0) return; // no parts to start the roof from!
    
    Polygons roofs = supporting.difference(to_be_supported);
    
    roofs = roofs.offset(-roof_inset).offset(roof_inset);
    
    if (roofs.size() == 0) return;
    
    
    Polygons roof_outlines; 
    Polygons roof_holes;
    { // split roofs into outlines and holes
        std::vector<Polygons> roof_parts = roofs.splitIntoParts();
        for (Polygons& roof_part : roof_parts)
        {
            roof_outlines.add(roof_part[0]);
            for (int hole_idx = 1; hole_idx < roof_part.size(); hole_idx++)
            {
                roof_holes.add(roof_part[hole_idx]);
                roof_holes.back().reverse();
            }
        }
    }

    
    Polygons supporting_outlines;
    
    std::vector<Polygons> supporting_parts = supporting.splitIntoParts();
    for (Polygons& supporting_part : supporting_parts) 
        supporting_outlines.add(supporting_part[0]); // only add outlines, not the holes
    
    
    
    Polygons inset1;
    Polygons last_inset;
    Polygons last_supported = supporting;
    for (Polygons inset0 = supporting_outlines; inset0.size() > 0; inset0 = last_inset)
    {
        last_inset = inset0.offset(direction * roof_inset, ClipperLib::jtRound);
        inset1 = last_inset.intersection(roof_outlines); // stay within roof area
        inset1 = inset1.unionPolygons(roof_holes);// make insets go around holes

        if (inset1.size() == 0) break;
        
        insets.emplace_back();
        
        connect(last_supported, z, inset1, z, insets.back(), true);
        
        inset1 = inset1.remove(roof_holes); // throw away holes which appear in every intersection
        inset1 = inset1.remove(roof_outlines);// throw away fully filled regions
        
        last_supported = insets.back().supported; // chainified
        
    }
    

    
    horizontals.roof_outlines.add(roofs); // TODO just add the new lines, not the lines of the roofs which are already supported ==> make outlines into a connection from which we only print the top, not the connection
} 
/*!
 * Fill floors starting from the outlines of \p supporting.
 * The area to be filled in is \p floors = difference( \p to_be_supported , \p supporting ).
 * 
 * The basic algorithm performs outsets until the whole area of [to_be_supported] is filled.
 * In order to not fill too much, the outsets are intersected with the [floors] area, which results in connections where the UP move has close to zero length.
 * These parts of the horizontal infills are converted into moves by the function [connections2moves].
 * 
 * The first supporting polygons are \p supporting while the supporting polygons in consecutive iterations are sub-areas of \p floors.
 * 
 * Note that the new outset is computed from the last outset, while the connections are between the last chainified outset and the new (chainified) outset.
 * 
 */
void Weaver::fillFloors(Polygons& supporting, Polygons& to_be_supported, int direction, int z, WeaveRoof& horizontals)
{
    std::vector<WeaveRoofPart>& outsets = horizontals.roof_insets;
    
    if (to_be_supported.size() == 0) return; // no parts to start the floor from!
    if (supporting.size() == 0) return; // no parts to start the floor from!
    
    Polygons floors = to_be_supported.difference(supporting);
    
    floors = floors.offset(-roof_inset).offset(roof_inset);
    
    if (floors.size() == 0) return;
    
    
    std::vector<Polygons> floor_parts = floors.splitIntoParts();
    
    Polygons floor_outlines;
    Polygons floor_holes;
    for (Polygons& floor_part : floor_parts)
    {
        floor_outlines.add(floor_part[0]);
        for (int hole_idx = 1; hole_idx < floor_part.size(); hole_idx++)
        {
            floor_holes.add(floor_part[hole_idx]);
            //floor_holes.back().reverse();
        }
    }
    
    
    Polygons outset1;
    
    Polygons last_supported = supporting;
    
    for (Polygons outset0 = supporting; outset0.size() > 0; outset0 = outset1)
    {
        outset1 = outset0.offset(roof_inset * direction, ClipperLib::jtRound).intersection(floors);
        outset1 = outset1.remove(floor_holes); // throw away holes which appear in every intersection
        outset1 = outset1.remove(floor_outlines); // throw away holes which appear in every intersection
        
        
        outsets.emplace_back();
        
        connect(last_supported, z, outset1, z, outsets.back(), true);
        
        outset1 = outset1.remove(floor_outlines);// throw away fully filled regions
        
        last_supported = outsets.back().supported; // chainified

    }
    
    
    horizontals.roof_outlines.add(floors);
}

/*!
 * Filter out parts of connections with small distances; replace by moves.
 * 
 */
void Weaver::connections2moves(WeaveRoofPart& inset)
{
    
    
    bool include_half_of_last_down = true;
    
    
    bool last_skipped = false;
    for (WeaveConnectionPart& part : inset.connections)
    {
        std::vector<WeaveConnectionSegment>& segments = part.connection.segments;
        for (int idx = 0; idx < part.connection.segments.size(); idx += 2)
        {
            WeaveConnectionSegment& segment = segments[idx];
            assert(segment.segmentType == WeaveSegmentType::UP);
            Point3 from = (idx == 0)? part.connection.from : segments[idx-1].to;
            bool skipped = (segment.to - from).vSize2() < extrusionWidth * extrusionWidth;
            if (skipped)
            {
                int begin = idx;
                for (; idx < segments.size(); idx += 2)
                {
                    WeaveConnectionSegment& segment = segments[idx];
                    assert(segments[idx].segmentType == WeaveSegmentType::UP);
                    Point3 from = (idx == 0)? part.connection.from : segments[idx-1].to;
                    bool skipped = (segment.to - from).vSize2() < extrusionWidth * extrusionWidth;
                    if (!skipped) 
                        break;
                }
                int end = idx - ((include_half_of_last_down)? 2 : 1);
                if (idx >= segments.size())
                    segments.erase(segments.begin() + begin, segments.end());
                else 
                {
                    segments.erase(segments.begin() + begin, segments.begin() + end);
                    if (begin < segments.size()) 
                    {
                        segments[begin].segmentType = WeaveSegmentType::MOVE;
                        if (include_half_of_last_down)
                            segments[begin+1].segmentType = WeaveSegmentType::DOWN_AND_FLAT;
                    }
                    idx = begin + ((include_half_of_last_down)? 2 : 1);
                }
            }
        }
    }
}

/*!
 * Connect two polygons, chainify the second and generate connections from it, supporting on the first polygon.
 * 
 * \param include_last Whether the last full link should be included in the chainified \p parts1 if the last link would be shorter than the normal link size.
 */
void Weaver::connect(Polygons& parts0, int z0, Polygons& parts1, int z1, WeaveConnection& result, bool include_last)
{
    // TODO: convert polygons (with outset + difference) such that after printing the first polygon, we can't be in the way of the printed stuff
    // something like:
    // for (m > n)
    //     parts[m] = parts[m].difference(parts[n].offset(nozzle_top_diameter))
    // according to the printing order!
    //
    // OR! : 
    //
    // unify different parts if gap is too small
    
    Polygons& supported = result.supported;
    
    if (parts1.size() == 0) return;
    
    Point& start_close_to = (parts0.size() > 0)? parts0.back().back() : parts1.back().back();
    
    chainify_polygons(parts1, start_close_to, supported, include_last);
    
    if (parts0.size() == 0) return;
    
    connect_polygons(parts0, z0, supported, z1, result);

}

/*!
 * Convert polygons, such that they consist of segments/links of uniform size, namely \p nozzle_top_diameter.
 * \param include_last governs whether the last segment is smaller or grater than the \p nozzle_top_diameter.
 * If true, the last segment may be smaller.
 */
void Weaver::chainify_polygons(Polygons& parts1, Point start_close_to, Polygons& result, bool include_last)
{
    
        
    for (int prt = 0 ; prt < parts1.size(); prt++)
    {
        const PolygonRef upperPart = parts1[prt];
        
        ClosestPolygonPoint closestInPoly = findClosest(start_close_to, upperPart);

        
        PolygonRef part_top = result.newPoly();
        
        GivenDistPoint next_upper;
        bool found = true;
        int idx = 0;
        
        for (Point upper_point = upperPart[closestInPoly.pos]; found; upper_point = next_upper.p)
        {
            found = getNextPointWithDistance(upper_point, nozzle_top_diameter, upperPart, idx, closestInPoly.pos, next_upper);

            
            if (!found) 
            {
                break;
            }
            
            part_top.add(upper_point);
            
            idx = next_upper.pos;
        }
        if (part_top.size() > 0)
            start_close_to = part_top.back();
        else
            result.remove(result.size()-1);
    }
}

/*!
 * The main weaving function.
 * Generate connections between two polygons.
 * The connections consist of triangles of which the first 
 */
void Weaver::connect_polygons(Polygons& supporting, int z0, Polygons& supported, int z1, WeaveConnection& result)
{
 
    if (supporting.size() < 1)
    {
        DEBUG_PRINTLN("lower layer has zero parts!");
        return;
    }
    
    result.z0 = z0;
    result.z1 = z1;
    
    std::vector<WeaveConnectionPart>& parts = result.connections;
        
    for (int prt = 0 ; prt < supported.size(); prt++)
    {
        
        const PolygonRef upperPart = supported[prt];
        
        
        parts.emplace_back(prt);
        WeaveConnectionPart& part = parts.back();
        PolyLine3& connection = part.connection;
        
        Point3 last_upper;
        bool firstIter = true;
        
        for (const Point& upper_point : upperPart)
        {
            
            ClosestPolygonPoint lowerPolyPoint = findClosest(upper_point, supporting);
            Point& lower = lowerPolyPoint.p;
            
            Point3 lower3 = Point3(lower.X, lower.Y, z0);
            Point3 upper3 = Point3(upper_point.X, upper_point.Y, z1);
            
            
            if (firstIter)
                connection.from = lower3;
            else
                connection.segments.emplace_back<>(lower3, WeaveSegmentType::DOWN);
            
            connection.segments.emplace_back<>(upper3, WeaveSegmentType::UP);
            last_upper = upper3;
            
            firstIter = false;
        }
    }
}











/*!
 * Find the point closest to \p from in all polygons in \p polygons.
 */
ClosestPolygonPoint Weaver::findClosest(Point from, Polygons& polygons)
{

    Polygon emptyPoly;
    ClosestPolygonPoint none(from, -1, emptyPoly);
    
    if (polygons.size() == 0) return none;
    PolygonRef aPolygon = polygons[0];
    if (aPolygon.size() == 0) return none;
    Point aPoint = aPolygon[0];

    ClosestPolygonPoint best(aPoint, 0, aPolygon);

    int64_t closestDist = vSize2(from - best.p);
    
    for (int ply = 0; ply < polygons.size(); ply++)
    {
        PolygonRef poly = polygons[ply];
        if (poly.size() == 0) continue;
        ClosestPolygonPoint closestHere = findClosest(from, poly);
        int64_t dist = vSize2(from - closestHere.p);
        if (dist < closestDist)
        {
            best = closestHere;
            closestDist = dist;
        }

    }

    return best;
}

/*!
 * Find the point closest to \p from in the polygon \p polygon.
 */
ClosestPolygonPoint Weaver::findClosest(Point from, PolygonRef polygon)
{
    Point aPoint = polygon[0];
    Point best = aPoint;

    int64_t closestDist = vSize2(from - best);
    int bestPos = 0;
//
    for (int p = 0; p<polygon.size(); p++)
    {
        Point& p1 = polygon[p];

        int p2_idx = p+1;
        if (p2_idx >= polygon.size()) p2_idx = 0;
        Point& p2 = polygon[p2_idx];

        Point closestHere = getClosestOnLine(from, p1 ,p2);
        int64_t dist = vSize2(from - closestHere);
        if (dist < closestDist)
        {
            best = closestHere;
            closestDist = dist;
            bestPos = p;
        }
    }

    return ClosestPolygonPoint(best, bestPos, polygon);
}

/*!
 * Find the point closest to \p from on the line from \p p0 to \p p1
 */
Point Weaver::getClosestOnLine(Point from, Point p0, Point p1)
{
    Point direction = p1 - p0;
    Point toFrom = from-p0;
    int64_t projected_x = dot(toFrom, direction) ;

    int64_t x_p0 = 0;
    int64_t x_p1 = vSize2(direction);

    if (projected_x <= x_p0)
    {
        return p0;
    }
    if (projected_x >= x_p1)
    {
        return p1;
    }
    else
    {
        if (vSize2(direction) == 0)
        {
            std::cout << "warning! too small segment" << std::endl;
            return p0;
        }
        Point ret = p0 + projected_x / vSize(direction) * direction  / vSize(direction);
        return ret ;
    }

}













/*!
 * Find the next point (going along the direction of the polygon) with a distance \p dist from the point \p from within the \p poly.
 * Returns whether another point could be found within the \p poly which can be found before encountering the point at index \p start_idx.
 * \param start_idx is the index of the prev poly point on the poly.
 * 
 * The point \p from and the polygon \p poly are assumed to lie on the same plane.
 */
bool Weaver::getNextPointWithDistance(Point from, int64_t dist, const PolygonRef poly, int start_idx, int poly_start_idx, GivenDistPoint& result)
{
    
    Point prev_poly_point = poly[(start_idx + poly_start_idx) % poly.size()];
    
    for (int prev_idx = start_idx; prev_idx < poly.size(); prev_idx++) 
    {
        int next_idx = (prev_idx + 1 + poly_start_idx) % poly.size(); // last checked segment is between last point in poly and poly[0]...
        Point& next_poly_point = poly[next_idx];
        if ( !shorterThen(next_poly_point - from, dist) )
        {
            /*
             *                 x    r
             *      p.---------+---+------------.n
             *                L|  /
             *                 | / dist
             *                 |/
             *                f.
             * 
             * f=from
             * p=prev_poly_point
             * n=next_poly_point
             * x= f projected on pn
             * r=result point at distance [dist] from f
             */
            
            Point pn = next_poly_point - prev_poly_point;
            
            if (shorterThen(pn, 100)) // when precision is limited
            {
                Point middle = (next_poly_point + prev_poly_point) / 2;
                int64_t dist_to_middle = vSize(from - middle);
                if (dist_to_middle - dist < 100 && dist_to_middle - dist > -100)
                {
                    result.p = middle;
                    result.pos = prev_idx;
                    return true;
                } else
                {
                    prev_poly_point = next_poly_point;
                    continue;
                }
            }
            
            Point pf = from - prev_poly_point;
            Point px = dot(pf, pn) / vSize(pn) * pn / vSize(pn);
            Point xf = pf - px;
            
            if (!shorterThen(xf, dist)) // line lies wholly further than pn
            {
                prev_poly_point = next_poly_point;
                continue;
                
            }
            
            int64_t xr_dist = std::sqrt(dist*dist - vSize2(xf)); // inverse Pythagoras
            
            if (vSize(pn - px) - xr_dist < 1) // r lies beyond n
            {
                prev_poly_point = next_poly_point;
                continue;
            }
            
            Point xr = xr_dist * pn / vSize(pn);
            Point pr = px + xr;
            
            result.p = prev_poly_point + pr;
            result.pos = prev_idx;
            return true;
        }
        prev_poly_point = next_poly_point;
    }
    return false;
}






    