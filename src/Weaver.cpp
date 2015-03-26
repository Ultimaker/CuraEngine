#include "Weaver.h"

#include <cmath> // sqrt
#include <fstream> // debug IO

#include "weaveDataStorage.h"

using namespace cura;


void Weaver::weave(PrintObject* object)
{
    int maxz = object->max().z;

    //bool succeeded = prepareModel(storage, model);

    int layer_count = (maxz - initial_layer_thickness) / connectionHeight + 1;

    DEBUG_SHOW(layer_count);

    std::vector<cura::Slicer*> slicerList;

    for(Mesh& mesh : object->meshes)
    {
        int fix_horrible = true; // mesh.getSettingInt("fixHorrible");
        cura::Slicer* slicer = new cura::Slicer(&mesh, initial_layer_thickness, connectionHeight, layer_count, false, false); // fix_horrible & FIX_HORRIBLE_KEEP_NONE_CLOSED, fix_horrible & FIX_HORRIBLE_EXTENSIVE_STITCHING);
        slicerList.push_back(slicer);
    }

    
    int starting_l;
    { // checking / verifying  (TODO: remove this code if error has never been seen!)
        for (starting_l = 0; starting_l < layer_count; starting_l++)
        {
            Polygons parts;
            for (cura::Slicer* slicer : slicerList)
                parts.add(slicer->layers[starting_l].polygonList);
            
            if (parts.size() > 0)
                break;
        }
        if (starting_l > 0)
        {
            logError("First %i layers are empty!\n", starting_l);
        }
    }
    
    
    std::cerr<< " chainifying layers..." << std::endl;
    {
        for (cura::Slicer* slicer : slicerList)
            wireFrame.bottom.add(getOuterPolygons(slicer->layers[starting_l].polygonList));
        
        wireFrame.z_bottom = slicerList[0]->layers[starting_l].z;
        
        for (int l = starting_l + 1; l < layer_count; l++)
        {
            DEBUG_PRINTLN(" layer : " << l);
            Polygons parts1;
            for (cura::Slicer* slicer : slicerList)
                parts1.add(getOuterPolygons(slicer->layers[l].polygonList));

            wireFrame.layers.emplace_back();
            WireLayer& layer = wireFrame.layers.back();
            
            layer.z0 = slicerList[0]->layers[l-1].z;
            layer.z1 = slicerList[0]->layers[l].z;
            
            chainify_polygons(parts1, layer.z1, layer.supported, false);
            
        }
    }
    
    std::cerr<< "finding roof parts..." << std::endl;
    {
        Polygons* lower_top_parts = &wireFrame.bottom;
        
        for (int l = 0; l < wireFrame.layers.size(); l++)
        {
            DEBUG_PRINTLN(" layer : " << l);
            WireLayer& layer = wireFrame.layers[l];
            
            Polygons empty;
            Polygons& layer_above = (l+1 < wireFrame.layers.size())? wireFrame.layers[l+1].supported : empty;
            
            createRoofs(*lower_top_parts, layer, layer_above, layer.z1);
            lower_top_parts = &layer.supported;
            
            
        }
    }
    
    
    std::cerr<< "connecting layers..." << std::endl;
    {
        Polygons* lower_top_parts = &wireFrame.bottom;
        int last_z = wireFrame.z_bottom;
        for (int l = 0; l < wireFrame.layers.size(); l++) // use top of every layer but the last
        {
            DEBUG_PRINTLN(" layer : " << l);
            WireLayer& layer = wireFrame.layers[l];
            
            connect_polygons(*lower_top_parts, last_z, layer.supported, layer.z1, layer);
            lower_top_parts = &layer.supported;
            
            last_z = layer.z1;
            
        }
    }

//     std::vector<WireLayer>& layers = wireFrame.layers;
//     for (int l = starting_l + 1; l < layer_count; l++)
//     {
//         Polygons& supported = layers[l].supported;
//         Polygons& 
//     }

    if (false)
    { // roofs:
        // TODO: introduce separate top roof layer
        // TODO: compute roofs for all layers!
        
        WireLayer& top_layer = wireFrame.layers.back();
        Polygons to_be_supported; // empty for the top layer
        fillRoofs(top_layer.supported, top_layer.z1, top_layer.roof_insets, to_be_supported);
    }
    
    // bottom:
    {
        Polygons to_be_supported; // empty for the bottom layer cause the order of insets doesn't really matter (in a sense everything is to be supported)
        fillRoofs(wireFrame.bottom, wireFrame.layers.front().z0, wireFrame.bottom_insets, to_be_supported);
    }
    
}


void Weaver::createRoofs(Polygons& lower_top_parts, WireLayer& layer, Polygons& layer_above, int z1)
{
    int64_t bridgable_dist = connectionHeight;
    
    Polygons& polys_below = lower_top_parts;
    Polygons& polys_here = layer.supported;
    Polygons& polys_above = layer_above;
    
    { // roofs
        Polygons to_be_supported =  polys_above.offset(bridgable_dist);
        fillRoofs(polys_here, z1, layer.roof_insets, to_be_supported);
        
        Polygons roof_outlines  = polys_here.difference(to_be_supported);
        layer.supported.add(roof_outlines);
    }
    
    
    { // floors
        Polygons to_be_supported =  polys_above.offset(-bridgable_dist);
        fillFloors(polys_here, z1, layer.roof_insets, to_be_supported);
        
        Polygons floor_outlines = polys_above.offset(-bridgable_dist).difference(polys_here);
        layer.supported.add(floor_outlines);
    }
    
}
    
template<class WireConnection_>
void Weaver::fillRoofs(Polygons& supporting, int z, std::vector<WireConnection_>& result, Polygons& to_be_supported)
{
    std::vector<Polygons> supporting_parts = supporting.splitIntoParts();
    
    Polygons supporting_outlines;
    Polygons holes;
    for (Polygons& supporting_part : supporting_parts)
    {
        supporting_outlines.add(supporting_part[0]);
        for (int hole_idx = 1; hole_idx < supporting_part.size(); hole_idx++)
        {
            holes.add(supporting_part[hole_idx]);
            holes.back().reverse();
        }
    }
    
    Polygons walk_along = holes.unionPolygons(to_be_supported);
    
    
    supporting_outlines = supporting_outlines.unionPolygons(to_be_supported);
    supporting_outlines = supporting_outlines.remove(to_be_supported);
    
    Polygons inset1;
    
    for (Polygons inset0 = supporting_outlines; inset0.size() > 0; inset0 = inset1)
    {
        Polygons simple_inset = inset0.offset(-roof_inset);
        simple_inset = simple_inset.unionPolygons(walk_along);
        inset1 = simple_inset.remove(walk_along); // only keep insets and inset-walk_along interactions (not pure walk_alongs!)
        
        if (inset1.size() == 0) break;
        
        result.emplace_back();
        
        connect(inset0, z, inset1, z, result.back(), true);
        
    }
}
     
    
template<class WireConnection_>
void Weaver::fillFloors(Polygons& supporting, int z, std::vector<WireConnection_>& result, Polygons& to_be_supported)
{
    std::vector<Polygons> to_be_supported_parts = to_be_supported.splitIntoParts();
    
    Polygons to_be_supported_outline;
    Polygons holes;
    for (Polygons& to_be_supported_part : to_be_supported_parts)
    {
        to_be_supported_outline.add(to_be_supported_part[0]);
        for (int hole_idx = 1; hole_idx < to_be_supported_part.size(); hole_idx++)
        {
            holes.add(to_be_supported_part[hole_idx]);
            holes.back().reverse();
        }
    }
    
    Polygons walk_along = holes.unionPolygons(to_be_supported);
    
    Polygons supporting_ = supporting;
    supporting_ = supporting_.intersection(to_be_supported);
    supporting_ = supporting_.remove(to_be_supported);
    
    Polygons outset1;
    
    for (Polygons outset0 = supporting_; outset0.size() > 0; outset0 = outset1)
    {
        Polygons simple_outset = outset0.offset(roof_inset);
        simple_outset = simple_outset.intersection(walk_along);
        outset1 = simple_outset.remove(walk_along); // only keep insets and inset-walk_along interactions (not pure walk_alongs!)
        
        if (outset1.size() == 0) break;
        
        result.emplace_back();
        
        connect(outset0, z, outset1, z, result.back(), true);
        
    }
}
     
Polygons Weaver::getOuterPolygons(Polygons& in)
{
    Polygons result;
    //getOuterPolygons(in, result);
    return in; // result; // TODO: 
}
void Weaver::getOuterPolygons(Polygons& in, Polygons& result)
{
    std::vector<Polygons> parts = in.splitIntoParts();
    for (Polygons& part : parts)
        result.add(part[0]);
    // TODO: remove parts inside of other parts
}


void Weaver::connect(Polygons& parts0, int z0, Polygons& parts1, int z1, WireConnection& result, bool include_last)
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
    chainify_polygons(parts1, z1, supported, include_last);
    connect_polygons(parts0, z0, supported, z1, result);
}


void Weaver::chainify_polygons(Polygons& parts1, int z, Polygons& top_parts, bool include_last)
{
        
    for (int prt = 0 ; prt < parts1.size(); prt++)
    {
        
        const PolygonRef upperPart = parts1[prt];
        
        
        PolygonRef part_top = top_parts.newPoly();
        
        GivenDistPoint next_upper;
        bool found = true;
        int idx = 0;
        Point3 last_upper;
        bool firstIter = true;
        for (Point upper_point = upperPart[0]; found; upper_point = next_upper.p)
        {
            found = getNextPointWithDistance(upper_point, nozzle_top_diameter, upperPart, z, idx, next_upper);
            
            if (!found) 
            {
                if (include_last)
                {
                    part_top.add(upper_point);
                }   
                break;
            }
            
            part_top.add(upper_point);
           
            last_upper = Point3(upper_point.X, upper_point.Y, z);
            
            
            idx = next_upper.pos;
            
            firstIter = false;
        }
    }
}
void Weaver::connect_polygons(Polygons& supporting, int z0, Polygons& supported, int z1, WireConnection& result)
{
 
    if (supporting.size() < 1)
    {
        DEBUG_PRINTLN("lower layer has zero parts!");
        return;
    }
    
    result.z0 = z0;
    result.z1 = z1;
    
    std::vector<WireConnectionPart>& parts = result.connections;
        
    for (int prt = 0 ; prt < supported.size(); prt++)
    {
        
        const PolygonRef upperPart = supported[prt];
        
        
        parts.emplace_back(prt);
        WireConnectionPart& part = parts.back();
        std::vector<WireConnectionSegment>& connection = part.connection;
        
        Point3 last_upper;
        bool firstIter = true;
        
        for (const Point& upper_point : upperPart)
        {
            
            ClosestPolygonPoint lowerPolyPoint = findClosest(upper_point, supporting);
            Point& lower = lowerPolyPoint.p;
            
            Point3 lower3 = Point3(lower.X, lower.Y, z0);
            Point3 upper3 = Point3(upper_point.X, upper_point.Y, z1);
            
            if (!firstIter)
                connection.emplace_back<>(last_upper, lower3, ExtrusionDirection::DOWN);
            
            connection.emplace_back<>(lower3 , upper3, ExtrusionDirection::UP);
            last_upper = upper3;
            
            firstIter = false;
        }
    }
}












ClosestPolygonPoint Weaver::findClosest(Point from, Polygons& polygons)
{

    PolygonRef aPolygon = polygons[0];
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
            //DEBUG_PRINTLN("(found better)");
        }

    }

    return best;
}

ClosestPolygonPoint Weaver::findClosest(Point from, PolygonRef polygon)
{
    //DEBUG_PRINTLN("find closest from polygon");
    Point aPoint = polygon[0];
    Point best = aPoint;

    int64_t closestDist = vSize2(from - best);
    int bestPos = 0;

//    DEBUG_PRINTLN("from:");
//    DEBUG_PRINTLN(from.x <<", "<<from.y<<","<<10000);
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
            //DEBUG_PRINTLN(" found better");
        }
    }

    //DEBUG_PRINTLN("found closest from polygon");
    return ClosestPolygonPoint(best, bestPos, polygon);
}

Point Weaver::getClosestOnLine(Point from, Point p0, Point p1)
{
    //DEBUG_PRINTLN("find closest on line segment");
    // line equation : p0 + x1 * (p1 - p0)
    Point direction = p1 - p0;
    // line equation : p0 + x/direction.vSize2() * direction
    Point toFrom = from-p0;
    int64_t projected_x = dot(toFrom, direction) ;

    int64_t x_p0 = 0;
    int64_t x_p1 = vSize2(direction);
//
//    DEBUG_PRINTLN(p0.x << ", " << p0.y << ", " << (p0-from).vSize());
//    Point3 pp = p0 + projected_x  / direction.vSize() * direction / direction.vSize();
//    DEBUG_PRINTLN(pp.x << ", " << pp.y << ", " << (pp-from).vSize());
//    DEBUG_PRINTLN(p1.x << ", " << p1.y << ", " << (p1-from).vSize());
//    DEBUG_PRINTLN("");
    if (projected_x <= x_p0)
    {
       // DEBUG_PRINTLN("found closest on line segment");
        return p0;
    }
    if (projected_x >= x_p1)
    {
       // DEBUG_PRINTLN("found closest on line segment");
        return p1;
    }
    else
    {
       // DEBUG_PRINTLN("found closest on line segment");
        if (vSize2(direction) == 0)
        {
            std::cout << "warning! too small segment" << std::endl;
            return p0;
        }
        Point ret = p0 + projected_x / vSize(direction) * direction  / vSize(direction);
        //DEBUG_PRINTLN("using projection");
        //DEBUG_SHOW(ret);
        
        return ret ;
    }

}














// start_idx is the index of the prev poly point on the poly
bool Weaver::getNextPointWithDistance(Point from, int64_t dist, const PolygonRef poly, int z_polygon, int start_idx, GivenDistPoint& result)
{
    Point prev_poly_point = poly[start_idx];
//     for (int i = 1; i < poly.size(); i++)
    for (int prev_idx = start_idx; prev_idx < poly.size(); prev_idx++) 
    {
//         int idx = (i + start_idx) % poly.size();
        int next_idx = (prev_idx+1) % poly.size(); // last checked segment is between last point in poly and poly[0]...
        Point& next_poly_point = poly[next_idx];
        if ( !shorterThen(next_poly_point - from, dist) )
        {
            /*
             *                f.
             *                 |\
             *                 | \ dist
             *                 |  \
             *      p.---------+---+------------.n
             *                 x    r
             * 
             * f=from
             * p=prev_poly_point
             * n=next_poly_point
             * x= f projected on pn
             * r=result point at distance [dist] from f
             */
            
            Point pn = next_poly_point - prev_poly_point;
            Point pf = from - prev_poly_point;
//             int64_t projected = dot(pf, pn) / vSize(pn);
            Point px = dot(pf, pn) / vSize(pn) * pn / vSize(pn);
            Point xf = pf - px;
            int64_t xr_dist = std::sqrt(dist*dist - vSize2(xf));
            Point xr = xr_dist * pn / vSize(pn);
            Point pr = px + xr;
            
            result.p = prev_poly_point + pr;
            if (xr_dist > 100000 || xr_dist < 0)
            {
                DEBUG_SHOW(from);
                DEBUG_SHOW(prev_poly_point);
                DEBUG_SHOW(next_poly_point);
                DEBUG_SHOW("");
                DEBUG_SHOW(vSize(from));
                DEBUG_SHOW(vSize(pn));
                DEBUG_SHOW(vSize2(xf));
                DEBUG_SHOW(vSize(pf));
                DEBUG_SHOW(dist);
                DEBUG_SHOW(dist*dist);
                DEBUG_SHOW(dist*dist - vSize2(xf));
                DEBUG_SHOW(xr_dist);
                DEBUG_SHOW(vSize(xr));
                DEBUG_SHOW(vSize(xf));
                DEBUG_SHOW(vSize(px));
                DEBUG_SHOW(vSize(pr));
                DEBUG_SHOW(result.p);
                std::exit(0);
            }
            result.pos = prev_idx;
            return true;
        }
        prev_poly_point = next_poly_point;
    }
    return false;
}






    