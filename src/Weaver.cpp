#include "Weaver.h"

#include <cmath> // sqrt
#include <fstream> // debug IO

using namespace cura;


void Weaver::weave(PrintObject* object)
{
    int maxz = object->max().z;

    Point3 min = object->min() ;
    std::cout << min << std::endl;
    std::cout << object->max() << std::endl;
    //bool succeeded = prepareModel(storage, model);

    int initial_layer_thickness = MM2INT(.3); // object->getSettingInt("initialLayerThickness");
    int layer_thickness = nozzle_head_distance; // object->getSettingInt("layerThickness");
    DEBUG_SHOW(nozzle_top_diameter);
    int layer_count = (maxz - (initial_layer_thickness - layer_thickness / 2)) / layer_thickness + 1;

    DEBUG_SHOW(layer_count);

    std::vector<cura::Slicer*> slicerList;

    for(Mesh& mesh : object->meshes)
    {
        for (MeshVertex& v : mesh.vertices)
            v.p -= min;

        int fix_horrible = true; // mesh.getSettingInt("fixHorrible");
        cura::Slicer* slicer = new cura::Slicer(&mesh, initial_layer_thickness - layer_thickness / 2, layer_thickness, layer_count, false, false); // fix_horrible & FIX_HORRIBLE_KEEP_NONE_CLOSED, fix_horrible & FIX_HORRIBLE_EXTENSIVE_STITCHING);
        slicerList.push_back(slicer);
        /*
        for(SlicerLayer& layer : slicer->layers)
        {
            //Reporting the outline here slows down the engine quite a bit, so only do so when debugging.
            //sendPolygons("outline", layer_nr, layer.z, layer.polygonList);
            //sendPolygons("openoutline", layer_nr, layer.openPolygonList);
        }
        */
    }


//    std::cout << "slicerList size = " << slicerList.size() << std::endl;
//    std::cout << "slicer size = " << slicerList[0]->layers.size() << std::endl;
//    for (cura::SlicerLayer& layer : slicerList[0]->layers)
//        std::cout << "layer size = " << layer.polygonList.size() << std::endl;

//    cura::SlicerLayer& slice = slicerList[0]->layers[1];
//    for (int l = 0; l < slice.polygonList.size(); l++)
//    {
//        std::cout << " polygon " << l << std::endl;
//        PolygonRef segment = slice.polygonList[l];
//        for (int p = 0; p < segment.size(); p++)
//            std::cout << segment[p].X << ", " << segment[p].Y << std::endl;
//    }
    //saveMeshToFile(object->meshes[0], "bs.stl");

    int starting_l;
    for (starting_l = 0; starting_l < layer_count; starting_l++)
    {
        Polygons parts;
        for (cura::Slicer* slicer : slicerList)
            parts.add(slicer->layers[starting_l].polygonList);
        if (parts.size() > 0)
            break;
    }
    
    
    
    
    Polygons lower_top_parts;
    for (cura::Slicer* slicer : slicerList)
        lower_top_parts.add(slicer->layers[starting_l].polygonList);
    
    WireFrame result;
    
    for (int l = starting_l + 1; l < layer_count; l++)
    {
        DEBUG_PRINTLN(" layer : " << l);
        Polygons parts2;
        for (cura::Slicer* slicer : slicerList)
            parts2.add(slicer->layers[l].polygonList);

        result.layers.emplace_back();
        WireLayer& layer = result.layers.back();
        
        connect(lower_top_parts, slicerList[0]->layers[l-1].z, parts2, slicerList[0]->layers[l].z, layer);
        lower_top_parts = layer.top;
    }

    
    
    
    
    
    DEBUG_PRINTLN("writing wireframe to CSV file!");
    
    std::ofstream bsCSV("bs.csv");
    
    for (WireLayer& layer : result.layers)
    {
        for (WireLayerPart& part : layer.parts)
        {
            std::vector<ExtrudeSegment>& connection = part.connection;
            for (ExtrudeSegment e : connection)
            {
                bsCSV << e.from.x << ", " << e.from.y << ", " << e.from.z << std::endl;
                bsCSV << e.to.x << ", " << e.to.y << ", " << e.to.z << std::endl;
                
            }
            PolygonRef part_top = layer.top[part.top_index];
            bsCSV << part_top[part_top.size()-1].X << ", "<< part_top[part_top.size()-1].Y <<", " << part.z1 << std::endl;
            for (int p = 0; p < part_top.size(); p++)
            {
                bsCSV << part_top[p].X << ", "<< part_top[p].Y <<", " << part.z1 << std::endl;
            }
        }
    }
    bsCSV.close();

}


void Weaver::connect(Polygons& parts1, int z0, Polygons& parts2, int z1, WireLayer& result)
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
    
    if (parts1.size() < 1)
    {
        DEBUG_PRINTLN("lower layer has zero parts!");
        return;
    }
            
    Polygons& top_parts = result.top;
    std::vector<WireLayerPart>& parts = result.parts;
        
    for (int prt = 0 ; prt < parts2.size(); prt++)
    {
        
        const PolygonRef upperPart = parts2[prt];
        
        
        PolygonRef part_top = top_parts.newPoly();
        parts.emplace_back(z0, z1, top_parts.size() - 1);
        WireLayerPart& part = parts.back();
        std::vector<ExtrudeSegment>& connection = part.connection;
        
        GivenDistPoint next_upper;
        bool found = true;
        int idx = 0;
        Point3 last_upper;
        bool firstIter = true;
        for (Point upper_point = upperPart[0]; found; upper_point = next_upper.p)
        {
            part_top.add(upper_point);
            
            found = getNextPointWithDistance(upper_point, nozzle_top_diameter, upperPart, z1, idx, next_upper);
            
            if (!found) 
            {
                break;
            }
            
            ClosestPolygonPoint lowerPolyPoint = findClosest(upper_point, parts1);
            Point& lower = lowerPolyPoint.p;
            
            Point3 lower3 = Point3(lower.X, lower.Y, z0);
            Point3 upper3 = Point3(upper_point.X, upper_point.Y, z1);
            
            if (!firstIter)
                connection.emplace_back<>(last_upper, lower3, ExtrusionDirection::DOWN);
            
            connection.emplace_back<>(lower3 , upper3, ExtrusionDirection::UP);
            last_upper = upper3;

            
            idx = next_upper.pos;
            
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
        //DEBUG_SHOW(dist);
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


/*
bool Weaver::findFirstPointWithDistance(Point3 from, int64_t dist, PolygonRef poly, int z_polygon, int startingPos, Point3 furtherThen, Point3& result)
{
    int pos = startingPos;
    bool found = false;
    do {
        int next = (pos + 1) % poly.size();
        Point3 a(poly[pos].X, poly[pos].Y, z_polygon);
        Point3 b(poly[next].X, poly[next].Y, z_polygon);

        found = getPointWithDistance(from, dist, a, b, furtherThen, result);
        if (found)
        {
            return true;
        }
        pos = next;
    } while (!found && pos != startingPos);



    return false;
}

int64_t Weaver::divide(Point3 a, Point3 b) //!< assumes the two vectors are in the same direction
{
    int64_t ret;
    int64_t xx = b.x*b.x;
    int64_t yy = b.y*b.y;
    int64_t zz = b.z*b.z;
    if (xx >= yy && xx >= zz)
        ret = a.x/ b.x;
    else if (yy >= zz)
        ret =  a.y / b.y;
    else
        ret = a.z / b.z;

    //TRIANGLE_INTERSECT_DEBUG_PRINTLN("\n dividing (" << a<<" / "<<b <<") = "<< ret);
    return ret;
};
*/



/*!
Computes the point [r] on a line [ab] with a given distance [d] to [c]
returns false in case there is no point on the line segment [ab] within distance [d].

Note that two points on a line may satisfy the equation.
Generally we take the first point (closest to [a]),
 but the take the second in case the first would be closer to [a] than the point [furtherThen], which is assumed to be on [ab]

*/
/*
bool Weaver::getPointWithDistance(Point3 c, int64_t d, Point3 a, Point3 b, Point3 furtherThen, Point3& result)
{
    int dz = c.z - a.z;
    int d2d2 = d*d - dz*dz;

    Point3 ab = b - a;
    Point3 ac = c - a;
    // line equation : p0 + x/ab.vSize() * ab/ab.vSize

    int64_t x_projected_c = ab.dot(ac);

    int64_t x_a = 0;
    int64_t x_b = ab.vSize2();

    if (x_projected_c <= x_a)
    {
       // DEBUG_PRINTLN("found closest on line segment");
        result = a;
        return false;
    }
    if (x_projected_c >= x_b)
    {
       // DEBUG_PRINTLN("found closest on line segment");
        result = b;
        return false;
    }

    Point3 ac_p = x_projected_c / ab.vSize() * ab / ab.vSize();

    Point3 cc_p = ac - ac_p;

    int64_t rc_p = std::sqrt(d2d2 - cc_p.vSize2());


    int64_t x_furtherThen = divide(furtherThen * ab.vSize() , ab ) * ab.vSize();

    int64_t x_r = x_projected_c - rc_p * ab.vSize();

    bool takeFurther = x_furtherThen > x_r;

    Point3 ar = ac_p * (1 + rc_p / ac_p.vSize() * ((takeFurther)? 1 : -1));

    result = a + ar;
    return true;

}
*/

