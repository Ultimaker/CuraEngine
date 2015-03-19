#include "Weaver.h"

#include <cmath> // sqrt
#include <fstream> // debug IO

#include "weaveDataStorage.h"

using namespace cura;


void Weaver::weave(PrintObject* object)
{
    int maxz = object->max().z;

    //bool succeeded = prepareModel(storage, model);

    int initial_layer_thickness = object->getSettingInt("initialLayerThickness");
    int layer_thickness = nozzle_head_distance; // object->getSettingInt("layerThickness");
    DEBUG_SHOW(nozzle_top_diameter);
    int layer_count = (maxz - initial_layer_thickness) / layer_thickness + 1;

    DEBUG_SHOW(layer_count);

    std::vector<cura::Slicer*> slicerList;

    for(Mesh& mesh : object->meshes)
    {
        int fix_horrible = true; // mesh.getSettingInt("fixHorrible");
        cura::Slicer* slicer = new cura::Slicer(&mesh, initial_layer_thickness, layer_thickness, layer_count, false, false); // fix_horrible & FIX_HORRIBLE_KEEP_NONE_CLOSED, fix_horrible & FIX_HORRIBLE_EXTENSIVE_STITCHING);
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
    
    
    
    for (cura::Slicer* slicer : slicerList)
        wireFrame.bottom.add(getOuterPolygons(slicer->layers[starting_l].polygonList));
    
    Polygons lower_top_parts = wireFrame.bottom;
    
    for (int l = starting_l + 1; l < layer_count; l++)
    {
        DEBUG_PRINTLN(" layer : " << l);
        Polygons parts2;
        for (cura::Slicer* slicer : slicerList)
            parts2.add(getOuterPolygons(slicer->layers[l].polygonList));

        wireFrame.layers.emplace_back();
        WireLayer& layer = wireFrame.layers.back();
        connect(lower_top_parts, slicerList[0]->layers[l-1].z, parts2, slicerList[0]->layers[l].z, layer);
        lower_top_parts = layer.supported;
    }

    { // roofs:
        // TODO: introduce separate top roof layer
        // TODO: compute roofs for all layers!
        
        
        WireLayer& roof_layer = wireFrame.layers.back();
        Polygons roofs = roof_layer.supported;
        std::vector<Polygons> roof_parts = roofs.splitIntoParts();
        
        Polygons roof_outlines;
        for (Polygons& roof_part : roof_parts)
            roof_outlines.add(roof_part[0]);
        Polygons holes;
        for (Polygons& roof_part : roof_parts)
            for (int hole_idx = 1; hole_idx < roof_part.size(); hole_idx++)
            {
               holes.add(roof_part[hole_idx]);
            }
        
        Polygons inset1;
        for (Polygons inset0 = roof_outlines; inset0.size() > 0; inset0 = inset1)
        {
            Polygons simple_inset = inset0.offset(-roof_inset);
            simple_inset = simple_inset.unionPolygons(holes);
            inset1 = simple_inset.remove(holes); // only keep inseets and inset-hole interactions (not pure holes!)
            
//             connect(inset0, );
            
        }
    }
    
    
    
    /*
    
    DEBUG_PRINTLN("writing wireframe to CSV file!");
    
    std::ofstream bsCSV("bs.csv");

    int z0 = slicerList[0]->layers[starting_l].z;
    for (PolygonRef bottom_part : wireFrame.bottom)
    {
        bsCSV << bottom_part[bottom_part.size()-1].X << ", "<< bottom_part[bottom_part.size()-1].Y <<", " << z0 << std::endl;
        for (int p = 0; p < bottom_part.size(); p++)
        {
            bsCSV << bottom_part[p].X << ", "<< bottom_part[p].Y <<", " << z0 << std::endl;
        }
    }
    
    for (WireLayer& layer : wireFrame.layers)
    {
        for (WireLayerPart& part : layer.parts)
        {
            std::vector<WeaveConnectionSegment>& connection = part.connection;
            for (WeaveConnectionSegment e : connection)
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
    */
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

void Weaver::connect(Polygons& parts1, int z0, Polygons& parts2, int z1, WireConnection& result)
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
            
    Polygons& top_parts = result.supported;
    std::vector<WireLayerPart>& parts = result.connections;
        
    for (int prt = 0 ; prt < parts2.size(); prt++)
    {
        
        const PolygonRef upperPart = parts2[prt];
        
        
        PolygonRef part_top = top_parts.newPoly();
        parts.emplace_back(z0, z1, top_parts.size() - 1);
        WireLayerPart& part = parts.back();
        std::vector<WireConnectionSegment>& connection = part.connection;
        
        GivenDistPoint next_upper;
        bool found = true;
        int idx = 0;
        Point3 last_upper;
        bool firstIter = true;
        for (Point upper_point = upperPart[0]; found; upper_point = next_upper.p)
        {
            
            found = getNextPointWithDistance(upper_point, nozzle_top_diameter, upperPart, z1, idx, next_upper);
            
            if (!found) 
            {
                break;
            }
            
            part_top.add(upper_point);
            
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









void Weaver::writeGCode(GCodeExport& gcode, CommandSocket* commandSocket, int& maxObjectHeight)
{

    if (commandSocket)
        commandSocket->beginGCode();
    
    if (!gcode.isOpened())
    {
        DEBUG_PRINTLN("gcode not opened!");
        return;
    }
    
    
    int initial_layer_thickness = getSettingInt("initialLayerThickness");
    int layer_thickness = nozzle_head_distance; // object->getSettingInt("layerThickness");
    
    int filament_diameter = getSettingInt("filamentDiameter");
    int flow = getSettingInt("filamentFlow");
    double filament_area = M_PI * (INT2MM(filament_diameter) / 2.0) * (INT2MM(filament_diameter) / 2.0);
    double extrusion_per_mm_connection = 0.1; //INT2MM(line_width) * INT2MM(layer_thickness) / filament_area * double(flow) / 100.0;
    double extrusion_per_mm_flat = 0.13; //INT2MM(line_width) * INT2MM(layer_thickness) / filament_area * double(flow) / 100.0;
    
    RetractionConfig retraction_config;
    retraction_config.amount = 500; //INT2MM(getSettingInt("retractionAmount"));
    retraction_config.primeAmount = 400;//INT2MM(getSettingInt("retractionPrimeAmount"));
    retraction_config.speed = getSettingInt("retractionSpeed");
    retraction_config.primeSpeed = getSettingInt("retractionPrimeSpeed");
    retraction_config.zHop = 0; //getSettingInt("retractionZHop");
    
    int bottomSpeed = 5;
    int moveSpeed = 40;
    int upSpeed = 3;
    int downSpeed = 3;
    int flatSpeed = 3;
    
    double wait_at_each_top_point = 0.1;
    
    double wait_at_bottom = 0.0;
    int up_dist_half_speed = 300;
    
    const int MOVE_TO_STRAIGHTEN = 0;
    double top_pause = 0.0;
    int top_jump_dist = 600;
    
    
    const int RETRACT_TO_STRAIGHTEN = 1;
    double top_retract_pause = 2.0;
    int retract_hop_dist = 1000;
    bool reserve_diagonal_direction = false;
    bool after_retract_hop = false;
    bool go_horizontal_first = true;
    bool lower_retract_start = true;
    
    int straighten_strategy = MOVE_TO_STRAIGHTEN;
    
    
    double go_back_to_last_top = true;
    int straight_first_when_going_down = 40; // %
    
    
    auto go_down = [&](WireConnectionSegment& segment, WireLayer& layer, WireLayerPart& part, int segment_idx) 
    { 
        if (go_back_to_last_top)
            gcode.writeMove(segment.from, downSpeed, 0);
        if (straight_first_when_going_down <= 0)
        {
            gcode.writeMove(segment.to, downSpeed, extrusion_per_mm_connection);
        } else 
        {
            Point3& to = segment.to;
            Point3& from = segment.from;
            Point3 vec = to - from;
            Point3 in_between = from + vec * straight_first_when_going_down / 100;
//                 Point in_between2D(in_between.x, in_between.y);
            Point3 up(in_between.x, in_between.y, from.z);
            gcode.writeMove(up, downSpeed, extrusion_per_mm_connection);
            int64_t orr_length = vec.vSize();/*
            int64_t new_length = (up - from).vSize() + (to - up).vSize() + 5;
            double enlargement = new_length / orr_length;*/
            gcode.writeMove(to, downSpeed, 0);
        }
        gcode.writeDelay(wait_at_bottom);
        if (up_dist_half_speed > 0)
        {
            
            gcode.writeMove(Point3(0,0,up_dist_half_speed) + gcode.getPosition(), upSpeed / 2, extrusion_per_mm_connection * 2);
        }
    };
        
    auto down_dir = [&](WireConnectionSegment& segment, WireLayer& layer, WireLayerPart& part, int segment_idx)
    {
        if (reserve_diagonal_direction)
        {
            gcode.writeMove(segment.to, moveSpeed, 0);
            gcode.writeMove(segment.from, downSpeed, extrusion_per_mm_connection);
            gcode.writeMove(Point3(segment.to.x, segment.to.y, segment.from.z), downSpeed, 0); // same z! 
            gcode.writeMove(segment.to, downSpeed, 0);
        } else 
            go_down(segment, layer, part, segment_idx);
    };
        
    auto move_to_straighten = [&](WireConnectionSegment& segment, WireLayer& layer, WireLayerPart& part, int segment_idx)
    {
        gcode.writeMove(segment.to, upSpeed, extrusion_per_mm_connection);
        Point3 next_vector;
        if (segment_idx + 1 < part.connection.size())
        {
            WireConnectionSegment& next_segment = part.connection[segment_idx+1];
            next_vector = next_segment.to - next_segment.from;
        } else
        {
            next_vector = (segment.to - layer.supported[part.top_index][0]) * -1;
        }
        Point next_dir_2D(next_vector.x, next_vector.y);
        next_dir_2D = next_dir_2D * top_jump_dist / vSize(next_dir_2D);
        Point3 next_dir (next_dir_2D.X / 2, next_dir_2D.Y / 2, -top_jump_dist);
        
        Point3 current_pos = gcode.getPosition();
        
        gcode.writeMove(current_pos - next_dir, upSpeed, 0);
        gcode.writeDelay(top_pause);
        gcode.writeMove(current_pos + next_dir_2D, upSpeed, 0);
    };
    auto retract_to_straighten = [&](WireConnectionSegment& segment)
    {
        Point3& to = segment.to;
        if (lower_retract_start)
        {
            Point3 vec = to - segment.from;
            Point3 lowering = vec * retract_hop_dist / 2 / vec.vSize();
            Point3 lower = to - lowering;
            gcode.writeMove(lower, upSpeed, extrusion_per_mm_connection);
            gcode.writeRetraction(&retraction_config, true);
            gcode.writeMove(to + lowering, upSpeed, 0);
            gcode.writeDelay(top_retract_pause);
            if (after_retract_hop)
                gcode.writeMove(to + Point3(0, 0, retract_hop_dist), flatSpeed, 0);
            
        } else 
        {
            gcode.writeMove(to, upSpeed, extrusion_per_mm_connection);
            gcode.writeRetraction(&retraction_config, true);
            gcode.writeMove(to + Point3(0, 0, retract_hop_dist), flatSpeed, 0);
            gcode.writeDelay(top_retract_pause);
            if (after_retract_hop)
                gcode.writeMove(to + Point3(0, 0, retract_hop_dist*3), flatSpeed, 0);
        }
    };
    auto handle_segment = [&](WireConnectionSegment& segment, WireLayer& layer, WireLayerPart& part, int segment_idx) 
    {
    if (straighten_strategy == MOVE_TO_STRAIGHTEN)
        {
            if (segment.dir == ExtrusionDirection::UP)
            {
                move_to_straighten(segment, layer, part, segment_idx);
            } else 
                go_down(segment, layer, part, segment_idx);
        } else if (straighten_strategy == RETRACT_TO_STRAIGHTEN)
        {
            if (segment.dir == ExtrusionDirection::UP)
            {
                retract_to_straighten(segment);
            } else 
            {
                down_dir(segment, layer, part, segment_idx);
            }
        }
    };
    
    
    
    
    // roofs:
    int roof_inset = nozzle_head_distance; // 45 degrees
    
    
//             for(SliceMeshStorage& mesh : storage.meshes)
//                 if (mesh.settings->hasSetting("printTemperature") && mesh.settings->getSettingInt("printTemperature") > 0)
//                     gcode.writeTemperatureCommand(mesh.settings->getSettingInt("extruderNr"), mesh.settings->getSettingInt("printTemperature"));
//             for(SliceMeshStorage& mesh : storage.meshes)
//                 if (mesh.settings->hasSetting("printTemperature") && mesh.settings->getSettingInt("printTemperature") > 0)
//                     gcode.writeTemperatureCommand(mesh.settings->getSettingInt("extruderNr"), mesh.settings->getSettingInt("printTemperature"), true);
    { // starting Gcode
        if (hasSetting("printTemperature") && getSettingInt("printTemperature") > 0)
            gcode.writeTemperatureCommand(getSettingInt("extruderNr"), getSettingInt("printTemperature"));
        if (hasSetting("bedTemperature") && getSettingInt("bedTemperature") > 0)
            gcode.writeLine("M190 S%d ;Bed temperature", static_cast<double>(getSettingInt("bedTemperature"))/100);
        
        gcode.writeCode(getSetting("startCode").c_str());
        if (gcode.getFlavor() == GCODE_FLAVOR_BFB)
        {
            gcode.writeComment("enable auto-retraction");
            gcode.writeLine("M227 S%d P%d", getSettingInt("retractionAmount") * 2560 / 1000, getSettingInt("retractionAmount") * 2560 / 1000);
        }
    }
    
    //maxObjectHeight = 100000; //wireFrame.layers.back().parts[0].z1; // TODO: allow for serial printing
    
            
    unsigned int totalLayers = wireFrame.layers.size();
    gcode.writeComment("Layer count: %d", totalLayers);    
    
    
    gcode.writeComment("LAYER:%d", 0);
    gcode.writeComment("TYPE:SKIRT");
//     Point& begin = wireFrame.bottom[0][0];
//     Point3 begin3D (begin.X, begin.Y, initial_layer_thickness);
//     gcode.writeMove(begin3D, moveSpeed, 0);
//     
    gcode.setZ(initial_layer_thickness);
//     gcode.writeComment("%i", __LINE__);
    
    for (PolygonRef bottom_part : wireFrame.bottom)
    {
        if (bottom_part.size() == 0) continue;
        gcode.writeMove(bottom_part[bottom_part.size()-1], moveSpeed, 0);
        for (Point& segment_to : bottom_part)
        {
            gcode.writeMove(segment_to, bottomSpeed, extrusion_per_mm_flat);
        }
    }
    
    
    for (int layer_nr = 0; layer_nr < wireFrame.layers.size(); layer_nr++)
    {
        logProgress("export", layer_nr+1, totalLayers);  
        if (commandSocket) commandSocket->sendProgress(2.0/3.0 + 1.0/3.0 * float(layer_nr) / float(totalLayers));
        
        WireLayer& layer = wireFrame.layers[layer_nr];
        
        gcode.writeComment("LAYER:%d", layer_nr+1);
        
        int fanSpeed = getSettingInt("fanSpeedMax");
        if (layer_nr == 0)
            fanSpeed = getSettingInt("fanSpeedMin");
        gcode.writeFanCommand(fanSpeed);
        
        for (int part_nr = 0; part_nr < layer.connections.size(); part_nr++)
        {
            WireLayerPart& part = layer.connections[part_nr];
            
            if (part.connection.size() == 0) continue;
            if (layer.supported[part.top_index].size() == 0) continue;
            // TODO: retraction
            
            gcode.writeComment("TYPE:SUPPORT"); // connection
            Point point_same_height(part.connection[0].from.x, part.connection[0].from.y);
            gcode.writeMove(point_same_height, moveSpeed, 0);
            gcode.writeMove(part.connection[0].from, moveSpeed, 0);
            for (int segment_idx = 0; segment_idx < part.connection.size(); segment_idx++)
            {
                WireConnectionSegment& segment = part.connection[segment_idx];
                handle_segment(segment, layer, part, segment_idx);
            }
            
            gcode.writeComment("TYPE:SKIRT"); // top
            int new_z = initial_layer_thickness + layer_thickness * (layer_nr + 1);
            gcode.setZ(new_z);
            maxObjectHeight = std::max(maxObjectHeight, new_z);
            PolygonRef top_part = layer.supported[part.top_index];
            for (int poly_point = 0; poly_point < top_part.size(); poly_point++)
            {
                gcode.writeMove(top_part[poly_point], flatSpeed, extrusion_per_mm_flat);
                gcode.writeDelay(wait_at_each_top_point);
            }
        }
        
     
        
    }
    
    gcode.setZ(maxObjectHeight);
    
    gcode.writeFanCommand(0);

    if (commandSocket)
    {
        if (gcode.isOpened())
        {
            gcode.finalize(maxObjectHeight, getSettingInt("moveSpeed"), getSetting("endCode").c_str());
            for(int e=0; e<MAX_EXTRUDERS; e++)
                gcode.writeTemperatureCommand(e, 0, false);
        }
        gcode.close();
        commandSocket->endSendSlicedObject();
        commandSocket->endGCode();
    }
    
    
    
    
}

