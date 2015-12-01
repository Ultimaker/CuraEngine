#include "Wireframe2gcode.h"

#include <cmath> // sqrt
#include <fstream> // debug IO

#include "weaveDataStorage.h"
#include "Progress.h"

#include "pathOrderOptimizer.h" // for skirt

namespace cura 
{


void Wireframe2gcode::writeGCode(CommandSocket* commandSocket)
{

    gcode.preSetup(wireFrame.meshgroup);
    
    if (commandSocket)
        commandSocket->beginGCode();
    
    processStartingCode(commandSocket);
    
    int maxObjectHeight;
    if (wireFrame.layers.empty())
    {
        maxObjectHeight = 0;
    }
    else
    {
        maxObjectHeight = wireFrame.layers.back().z1;
    }
    
    processSkirt(commandSocket);
    
            
    unsigned int total_layers = wireFrame.layers.size();
    gcode.writeLayerComment(0);
    gcode.writeTypeComment("SKIRT");

    gcode.setZ(initial_layer_thickness);
    
    for (PolygonRef bottom_part : wireFrame.bottom_infill.roof_outlines)
    {
        if (bottom_part.size() == 0) continue;
        writeMoveWithRetract(bottom_part[bottom_part.size()-1]);
        for (Point& segment_to : bottom_part)
        {
            gcode.writeMove(segment_to, speedBottom, extrusion_per_mm_flat);
        }
    }
    
    
    
    // bottom:
    Polygons empty_outlines;
    writeFill(wireFrame.bottom_infill.roof_insets, empty_outlines, 
              [this](Wireframe2gcode& thiss, WeaveRoofPart& inset, WeaveConnectionPart& part, unsigned int segment_idx) { 
                    WeaveConnectionSegment& segment = part.connection.segments[segment_idx]; 
                    if (segment.segmentType == WeaveSegmentType::MOVE || segment.segmentType == WeaveSegmentType::DOWN_AND_FLAT) // this is the case when an inset overlaps with a hole 
                    {
                        writeMoveWithRetract(segment.to); 
                    } else 
                    {
                        gcode.writeMove(segment.to, speedBottom, extrusion_per_mm_connection); 
                    }
                }   
            , 
              [this](Wireframe2gcode& thiss, WeaveConnectionSegment& segment) { 
                    if (segment.segmentType == WeaveSegmentType::MOVE)
                        writeMoveWithRetract(segment.to);
                    else if (segment.segmentType == WeaveSegmentType::DOWN_AND_FLAT)
                        return; // do nothing
                    else 
                        gcode.writeMove(segment.to, speedBottom, extrusion_per_mm_flat); 
                }
            );
    Progress::messageProgressStage(Progress::Stage::EXPORT, nullptr, commandSocket);
    for (unsigned int layer_nr = 0; layer_nr < wireFrame.layers.size(); layer_nr++)
    {
        Progress::messageProgress(Progress::Stage::EXPORT, layer_nr+1, total_layers, commandSocket); // abuse the progress system of the normal mode of CuraEngine
        
        WeaveLayer& layer = wireFrame.layers[layer_nr];
        
        gcode.writeLayerComment(layer_nr+1);
        
        double fanSpeed = getSettingInPercentage("cool_fan_speed_max");
        if (layer_nr == 0)
            fanSpeed = getSettingInPercentage("cool_fan_speed_min");
        gcode.writeFanCommand(fanSpeed);
        
        for (unsigned int part_nr = 0; part_nr < layer.connections.size(); part_nr++)
        {
            WeaveConnectionPart& part = layer.connections[part_nr];
       
            if (part.connection.segments.size() == 0) continue;
            
            gcode.writeTypeComment("SUPPORT"); // connection
            {
                if (vSize2(gcode.getPositionXY() - part.connection.from) > connectionHeight)
                {
                    Point3 point_same_height(part.connection.from.x, part.connection.from.y, layer.z1+100);
                    writeMoveWithRetract(point_same_height);
                }
                writeMoveWithRetract(part.connection.from);
                for (unsigned int segment_idx = 0; segment_idx < part.connection.segments.size(); segment_idx++)
                {
                    handle_segment(layer, part, segment_idx);
                }
            }
            
            
            
            gcode.writeTypeComment("WALL-OUTER"); // top
            {
                for (unsigned int segment_idx = 0; segment_idx < part.connection.segments.size(); segment_idx++)
                {
                    WeaveConnectionSegment& segment = part.connection.segments[segment_idx];
                    if (segment.segmentType == WeaveSegmentType::DOWN) continue;
                    if (segment.segmentType == WeaveSegmentType::MOVE) 
                    {
                        writeMoveWithRetract(segment.to);
                    } else 
                    {
                        gcode.writeMove(segment.to, speedFlat, extrusion_per_mm_flat);
                        gcode.writeDelay(flat_delay);
                    }
                }
            }
        }
        
        // roofs:
        gcode.setZ(layer.z1);
        std::function<void (Wireframe2gcode& thiss, WeaveRoofPart& inset, WeaveConnectionPart& part, unsigned int segment_idx)>
            handle_roof = &Wireframe2gcode::handle_roof_segment;
        writeFill(layer.roofs.roof_insets, layer.roofs.roof_outlines,
                  handle_roof,
                [this](Wireframe2gcode& thiss, WeaveConnectionSegment& segment) { // handle flat segments
                    if (segment.segmentType == WeaveSegmentType::MOVE)
                    {
                        writeMoveWithRetract(segment.to);
                    } else if (segment.segmentType == WeaveSegmentType::DOWN_AND_FLAT)
                    {
                        // do nothing
                    } else 
                    {   
                        gcode.writeMove(segment.to, speedFlat, extrusion_per_mm_flat);
                        gcode.writeDelay(flat_delay);
                    }
                });
        
     
        
    }
    
    gcode.setZ(maxObjectHeight);
    
    gcode.writeRetraction(&standard_retraction_config);
    
    
    gcode.updateTotalPrintTime();
    
    gcode.writeDelay(0.3);
    
    gcode.writeFanCommand(0);

    finalize();
    
    if (commandSocket)
    {
        commandSocket->sendGCodeLayer();
        commandSocket->endSendSlicedObject();
    }
}

    
void Wireframe2gcode::go_down(WeaveLayer& layer, WeaveConnectionPart& part, unsigned int segment_idx) 
{ 
    WeaveConnectionSegment& segment = part.connection.segments[segment_idx];
    Point3 from = (segment_idx == 0)? part.connection.from : part.connection.segments[segment_idx - 1].to;
    if (go_back_to_last_top)
        gcode.writeMove(from, speedDown, 0);
    if (straight_first_when_going_down <= 0)
    {
        gcode.writeMove(segment.to, speedDown, extrusion_per_mm_connection);
    } else 
    {
        Point3& to = segment.to;
        Point3 from = gcode.getPosition();// segment.from;
        Point3 vec = to - from;
        Point3 in_between = from + vec * straight_first_when_going_down / 100;
        
        Point3 up(in_between.x, in_between.y, from.z);
        int64_t new_length = (up - from).vSize() + (to - up).vSize() + 5;
        int64_t orr_length = vec.vSize();
        double enlargement = new_length / orr_length;
        gcode.writeMove(up, speedDown*enlargement, extrusion_per_mm_connection / enlargement);
        gcode.writeMove(to, speedDown*enlargement, extrusion_per_mm_connection / enlargement);
    }
    gcode.writeDelay(bottom_delay);
    if (up_dist_half_speed > 0)
    {
        
        gcode.writeMove(Point3(0,0,up_dist_half_speed) + gcode.getPosition(), speedUp / 2, extrusion_per_mm_connection * 2);
    }
}


    
void Wireframe2gcode::strategy_knot(WeaveLayer& layer, WeaveConnectionPart& part, unsigned int segment_idx)
{
    WeaveConnectionSegment& segment = part.connection.segments[segment_idx];
    gcode.writeMove(segment.to, speedUp, extrusion_per_mm_connection);
    Point3 next_vector;
    if (segment_idx + 1 < part.connection.segments.size())
    {
        WeaveConnectionSegment& next_segment = part.connection.segments[segment_idx+1];
        next_vector = next_segment.to - segment.to;
    } else
    {
        next_vector = part.connection.segments[0].to - segment.to;
    }
    Point next_dir_2D(next_vector.x, next_vector.y);
    next_dir_2D = next_dir_2D * top_jump_dist / vSize(next_dir_2D);
    Point3 next_dir (next_dir_2D.X / 2, next_dir_2D.Y / 2, -top_jump_dist);
    
    Point3 current_pos = gcode.getPosition();
    
    gcode.writeMove(current_pos - next_dir, speedUp, 0);
    gcode.writeDelay(top_delay);
    gcode.writeMove(current_pos + next_dir_2D, speedUp, 0);
}

void Wireframe2gcode::strategy_retract(WeaveLayer& layer, WeaveConnectionPart& part, unsigned int segment_idx)
{
    WeaveConnectionSegment& segment = part.connection.segments[segment_idx];
    Point3 from = (segment_idx == 0)? part.connection.from : part.connection.segments[segment_idx - 1].to;
    
    RetractionConfig retraction_config;
    // TODO: get these from the settings!
    retraction_config.distance = 500; //INT2MM(getSettingInt("retraction_amount"))
    retraction_config.prime_volume = 0;//INT2MM(getSettingInt("retractionPrime
    retraction_config.speed = 20; // 40;
    retraction_config.primeSpeed = 15; // 30;
    retraction_config.zHop = 0; //getSettingInt("retraction_hop");
    retraction_config.retraction_count_max = getSettingAsCount("retraction_count_max");
    retraction_config.retraction_extrusion_window = INT2MM(getSettingInMicrons("retraction_extrusion_window"));
    retraction_config.retraction_min_travel_distance = getSettingInMicrons("retraction_min_travel");

    double top_retract_pause = 2.0;
    int retract_hop_dist = 1000;
    bool after_retract_hop = false;
    //bool go_horizontal_first = true;
    bool lower_retract_start = true;
    
    
    Point3& to = segment.to;
    if (lower_retract_start)
    {
        Point3 vec = to - from;
        Point3 lowering = vec * retract_hop_dist / 2 / vec.vSize();
        Point3 lower = to - lowering;
        gcode.writeMove(lower, speedUp, extrusion_per_mm_connection);
        gcode.writeRetraction(&retraction_config);
        gcode.writeMove(to + lowering, speedUp, 0);
        gcode.writeDelay(top_retract_pause);
        if (after_retract_hop)
            gcode.writeMove(to + Point3(0, 0, retract_hop_dist), speedFlat, 0);
        
    } else 
    {
        gcode.writeMove(to, speedUp, extrusion_per_mm_connection);
        gcode.writeRetraction(&retraction_config);
        gcode.writeMove(to + Point3(0, 0, retract_hop_dist), speedFlat, 0);
        gcode.writeDelay(top_retract_pause);
        if (after_retract_hop)    
            gcode.writeMove(to + Point3(0, 0, retract_hop_dist*3), speedFlat, 0);
        }
}

void Wireframe2gcode::strategy_compensate(WeaveLayer& layer, WeaveConnectionPart& part, unsigned int segment_idx)
{
    WeaveConnectionSegment& segment = part.connection.segments[segment_idx];
    Point3 from = (segment_idx == 0)? part.connection.from : part.connection.segments[segment_idx - 1].to;
    Point3 to = segment.to + Point3(0, 0, fall_down*(segment.to - from).vSize() / connectionHeight);
    Point3 vector = segment.to - from;
    Point3 dir = vector * drag_along / vector.vSize();
    
    Point3 next_point;
    if (segment_idx + 1 < part.connection.segments.size())
    {
        WeaveConnectionSegment& next_segment = part.connection.segments[segment_idx+1];
        next_point = next_segment.to;
    } else
    {
        next_point = part.connection.segments[0].to;
    }
    Point3 next_vector = next_point - segment.to;
    Point next_dir_2D(next_vector.x, next_vector.y);
    int64_t next_dir_2D_size = vSize(next_dir_2D);
    if (next_dir_2D_size > 0)
        next_dir_2D = next_dir_2D * drag_along / next_dir_2D_size;
    Point3 next_dir (next_dir_2D.X, next_dir_2D.Y, 0);
    
    Point3 newTop = to - next_dir + dir;
    
    int64_t orrLength = (segment.to - from).vSize() + next_vector.vSize() + 1; // + 1 in order to avoid division by zero
    int64_t newLength = (newTop - from).vSize() + (next_point - newTop).vSize() + 1; // + 1 in order to avoid division by zero
    
    gcode.writeMove(newTop, speedUp * newLength / orrLength, extrusion_per_mm_connection * orrLength / newLength);
}
void Wireframe2gcode::handle_segment(WeaveLayer& layer, WeaveConnectionPart& part, unsigned int segment_idx) 
{
    WeaveConnectionSegment& segment = part.connection.segments[segment_idx];
    
    switch(segment.segmentType)
    {
        case WeaveSegmentType::MOVE:
            writeMoveWithRetract(segment.to);
            break;
        case WeaveSegmentType::DOWN:
            go_down(layer, part, segment_idx);
            break;
        case WeaveSegmentType::FLAT:
            DEBUG_SHOW("flat piece in connection?!!?!");
            break;
        case WeaveSegmentType::UP:
            if (strategy == STRATEGY_KNOT)
            {
                strategy_knot(layer, part, segment_idx);
            } else if (strategy == STRATEGY_RETRACT)
            { 
                strategy_retract(layer, part, segment_idx);
            } else if (strategy == STRATEGY_COMPENSATE)
            {
                strategy_compensate(layer, part, segment_idx);
            }
            break;
        case WeaveSegmentType::DOWN_AND_FLAT:
            logError("Down and flat move in non-horizontal connection!");
            break;
    }
}




void Wireframe2gcode::handle_roof_segment(WeaveRoofPart& inset, WeaveConnectionPart& part, unsigned int segment_idx)
{
    WeaveConnectionSegment& segment = part.connection.segments[segment_idx];
    Point3 from = (segment_idx == 0)? part.connection.from : part.connection.segments[segment_idx - 1].to;
    WeaveConnectionSegment* next_segment = nullptr;
    if (segment_idx + 1 < part.connection.segments.size())
        next_segment = &part.connection.segments[segment_idx+1];
    switch(segment.segmentType)
    {
        case WeaveSegmentType::MOVE:
        case WeaveSegmentType::DOWN_AND_FLAT:
            if (next_segment && next_segment->segmentType != WeaveSegmentType::DOWN_AND_FLAT)
            {
                writeMoveWithRetract(segment.to);
            }
            break;
        case WeaveSegmentType::UP:
            {
                Point3 to = segment.to + Point3(0, 0, roof_fall_down);
                
                Point3 vector = segment.to - from;
                if (vector.vSize2() == 0) return;
                Point3 dir = vector * roof_drag_along / vector.vSize();
                
                Point3 next_vector;
                if (next_segment)
                {
                    next_vector = next_segment->to - segment.to;
                } else
                {
                    next_vector = part.connection.segments[0].to - segment.to;
                }
                Point next_dir_2D(next_vector.x, next_vector.y);
                Point3 detoured = to + dir;
                if (vSize2(next_dir_2D) > 0) 
                {
                    next_dir_2D = next_dir_2D * roof_drag_along / vSize(next_dir_2D);
                    Point3 next_dir (next_dir_2D.X, next_dir_2D.Y, 0);
                    detoured -= next_dir;
                }
                
                gcode.writeMove(detoured, speedUp, extrusion_per_mm_connection);

            }
            break;
        case WeaveSegmentType::DOWN:
            gcode.writeMove(segment.to, speedDown, extrusion_per_mm_connection);
            gcode.writeDelay(roof_outer_delay);
            break;
        case WeaveSegmentType::FLAT:
            logError("Flat move in connection!");
            break;
    }

}



void Wireframe2gcode::writeFill(std::vector<WeaveRoofPart>& infill_insets, Polygons& roof_outlines
    , std::function<void (Wireframe2gcode& thiss, WeaveRoofPart& inset, WeaveConnectionPart& part, unsigned int segment_idx)> connectionHandler
    , std::function<void (Wireframe2gcode& thiss, WeaveConnectionSegment& p)> flatHandler)
{
        
    // bottom:
    gcode.writeTypeComment("FILL");
    for (unsigned int inset_idx = 0; inset_idx < infill_insets.size(); inset_idx++)
    {
        WeaveRoofPart& inset = infill_insets[inset_idx];
        
        
        for (unsigned int inset_part_nr = 0; inset_part_nr < inset.connections.size(); inset_part_nr++)
        {
            WeaveConnectionPart& inset_part = inset.connections[inset_part_nr];
            std::vector<WeaveConnectionSegment>& segments = inset_part.connection.segments;
            
            gcode.writeTypeComment("SUPPORT"); // connection
            if (segments.size() == 0) continue;
            Point3 first_extrusion_from = inset_part.connection.from;
            unsigned int first_segment_idx;
            for (first_segment_idx = 0; first_segment_idx < segments.size() && segments[first_segment_idx].segmentType == WeaveSegmentType::MOVE; first_segment_idx++)
            { // finds the first segment which is not a move
                first_extrusion_from = segments[first_segment_idx].to;
            }
            if (first_segment_idx == segments.size())
                continue;
            writeMoveWithRetract(first_extrusion_from);
            for (unsigned int segment_idx = first_segment_idx; segment_idx < segments.size(); segment_idx++)
            {
                connectionHandler(*this, inset, inset_part, segment_idx);
            }
            
            gcode.writeTypeComment("WALL-INNER"); // top
            for (unsigned int segment_idx = 0; segment_idx < segments.size(); segment_idx++)
            {
                WeaveConnectionSegment& segment = segments[segment_idx];

                if (segment.segmentType == WeaveSegmentType::DOWN) continue;

                flatHandler(*this, segment); 
            }
        }
        
        
    }
    
    gcode.writeTypeComment("WALL-OUTER"); // outer perimeter of the flat parts
    for (PolygonRef poly : roof_outlines)
    {
        writeMoveWithRetract(poly[poly.size() - 1]);
        for (Point& p : poly)
        {
            Point3 to(p.X, p.Y, gcode.getPositionZ());
            WeaveConnectionSegment segment(to, WeaveSegmentType::FLAT);
            flatHandler(*this, segment); 
        }
    }
}




void Wireframe2gcode::writeMoveWithRetract(Point3 to)
{
    if ((gcode.getPosition() - to).vSize2() >= nozzle_top_diameter * nozzle_top_diameter * 2 * 2)
        gcode.writeRetraction(&standard_retraction_config);
    gcode.writeMove(to, moveSpeed, 0);
}

void Wireframe2gcode::writeMoveWithRetract(Point to)
{
    if (vSize2(gcode.getPositionXY() - to) >= nozzle_top_diameter * nozzle_top_diameter * 2 * 2)
        gcode.writeRetraction(&standard_retraction_config);
    gcode.writeMove(to, moveSpeed, 0);
}

Wireframe2gcode::Wireframe2gcode(Weaver& weaver, GCodeExport& gcode, SettingsBase* settings_base) 
: SettingsMessenger(settings_base) 
, gcode(gcode)
{
    wireFrame = weaver.wireFrame;
    initial_layer_thickness = getSettingInMicrons("layer_height_0");
    connectionHeight = getSettingInMicrons("wireframe_height"); 
    roof_inset = getSettingInMicrons("wireframe_roof_inset"); 
    
    filament_diameter = getSettingInMicrons("material_diameter");
    extrusionWidth = getSettingInMicrons("wall_line_width_x");
    
    flowConnection = getSettingInPercentage("wireframe_flow_connection");
    flowFlat = getSettingInPercentage("wireframe_flow_flat");
    
    double filament_area = /* M_PI * */ (INT2MM(filament_diameter) / 2.0) * (INT2MM(filament_diameter) / 2.0);
    double lineArea = /* M_PI * */ (INT2MM(extrusionWidth) / 2.0) * (INT2MM(extrusionWidth) / 2.0);
    extrusion_per_mm_connection = lineArea / filament_area * flowConnection / 100.0;
    extrusion_per_mm_flat = lineArea / filament_area * flowFlat / 100.0;
    
    nozzle_outer_diameter = getSettingInMicrons("machine_nozzle_tip_outer_diameter"); // ___       ___   .
    nozzle_head_distance = getSettingInMicrons("machine_nozzle_head_distance");      //    |     |      .
    nozzle_expansion_angle = getSettingInAngleRadians("machine_nozzle_expansion_angle");  //     \_U_/       .
    nozzle_clearance = getSettingInMicrons("wireframe_nozzle_clearance");    // at least line width
    nozzle_top_diameter = tan(nozzle_expansion_angle) * connectionHeight + nozzle_outer_diameter + nozzle_clearance;
    
    moveSpeed = 40;
    speedBottom =  getSettingInMillimetersPerSecond("wireframe_printspeed_bottom");
    speedUp = getSettingInMillimetersPerSecond("wireframe_printspeed_up");
    speedDown = getSettingInMillimetersPerSecond("wireframe_printspeed_down");
    speedFlat = getSettingInMillimetersPerSecond("wireframe_printspeed_flat");

    flat_delay = getSettingInSeconds("wireframe_flat_delay");
    bottom_delay = getSettingInSeconds("wireframe_bottom_delay");
    top_delay = getSettingInSeconds("wireframe_top_delay");
    
    up_dist_half_speed = getSettingInMicrons("wireframe_up_half_speed");
    
    top_jump_dist = getSettingInMicrons("wireframe_top_jump");
    
    fall_down = getSettingInMicrons("wireframe_fall_down");
    drag_along = getSettingInMicrons("wireframe_drag_along");
    
    strategy = STRATEGY_COMPENSATE;
    if (getSettingString("wireframe_strategy") == "Compensate")
        strategy = STRATEGY_COMPENSATE;
    if (getSettingString("wireframe_strategy") == "Knot")
        strategy = STRATEGY_KNOT;
    if (getSettingString("wireframe_strategy") == "Retract")
        strategy = STRATEGY_RETRACT;
    
    go_back_to_last_top = false;
    straight_first_when_going_down = getSettingInPercentage("wireframe_straight_before_down");
    
    roof_fall_down = getSettingInMicrons("wireframe_roof_fall_down");
    roof_drag_along = getSettingInMicrons("wireframe_roof_drag_along");
    roof_outer_delay = getSettingInSeconds("wireframe_roof_outer_delay");
    
    
    standard_retraction_config.distance = INT2MM(getSettingInMicrons("retraction_amount"));
    standard_retraction_config.prime_volume = getSettingInCubicMillimeters("retraction_extra_prime_amount");
    standard_retraction_config.speed = getSettingInMillimetersPerSecond("retraction_retract_speed");
    standard_retraction_config.primeSpeed = getSettingInMillimetersPerSecond("retraction_prime_speed");
    standard_retraction_config.zHop = getSettingInMicrons("retraction_hop");
    standard_retraction_config.retraction_count_max = getSettingAsCount("retraction_count_max");
    standard_retraction_config.retraction_extrusion_window = INT2MM(getSettingInMicrons("retraction_extrusion_window"));
    standard_retraction_config.retraction_min_travel_distance = getSettingInMicrons("retraction_min_travel");
}

void Wireframe2gcode::processStartingCode(CommandSocket* command_socket)
{
    if (gcode.getFlavor() == EGCodeFlavor::ULTIGCODE)
    {
        if (!command_socket)
        {
            gcode.writeCode(";FLAVOR:UltiGCode\n;TIME:666\n;MATERIAL:666\n;MATERIAL2:-1\n");
        }
    }
    else 
    {
        if (getSettingBoolean("material_bed_temp_prepend"))
        {
            if (getSettingBoolean("machine_heated_bed") && getSettingInDegreeCelsius("material_bed_temperature") > 0)
            {
                gcode.writeBedTemperatureCommand(getSettingInDegreeCelsius("material_bed_temperature"), getSettingBoolean("material_bed_temp_wait"));
            }
        }
        
        if (getSettingBoolean("material_print_temp_prepend"))
        {
            if (getSettingInDegreeCelsius("material_print_temperature") > 0)
            {
                gcode.writeTemperatureCommand(getSettingAsIndex("extruder_nr"), getSettingInDegreeCelsius("material_print_temperature"));
                if (getSettingBoolean("machine_print_temp_wait"))
                {
                    gcode.writeTemperatureCommand(getSettingAsIndex("extruder_nr"), getSettingInDegreeCelsius("material_print_temperature"), true);
                }
            }
        }
        
    }
    gcode.writeCode(getSettingString("machine_start_gcode").c_str());
    
    gcode.writeComment("Generated with Cura_SteamEngine " VERSION);
    if (gcode.getFlavor() == EGCodeFlavor::BFB)
    {
        gcode.writeComment("enable auto-retraction");
        std::ostringstream tmp;
        tmp << "M227 S" << (getSettingInMicrons("retraction_amount") * 2560 / 1000) << " P" << (getSettingInMicrons("retraction_amount") * 2560 / 1000);
        gcode.writeLine(tmp.str().c_str());
    }
}


void Wireframe2gcode::processSkirt(CommandSocket* commandSocket)
{
    if (wireFrame.bottom_outline.size() == 0) //If we have no layers, don't create a skirt either.
    {
        return;
    }
    Polygons skirt = wireFrame.bottom_outline.offset(100000+5000).offset(-100000);
    PathOrderOptimizer order(Point(INT32_MIN, INT32_MIN));
    order.addPolygons(skirt);
    order.optimize();
    
    for (unsigned int poly_idx = 0; poly_idx < skirt.size(); poly_idx++)
    {
        unsigned int actual_poly_idx = order.polyOrder[poly_idx];
        PolygonRef poly = skirt[actual_poly_idx];
        gcode.writeMove(poly[order.polyStart[actual_poly_idx]], getSettingInMillimetersPerSecond("speed_travel"), 0);
        for (unsigned int point_idx = 0; point_idx < poly.size(); point_idx++)
        {
            Point& p = poly[(point_idx + order.polyStart[actual_poly_idx] + 1) % poly.size()];
            gcode.writeMove(p, getSettingInMillimetersPerSecond("skirt_speed"), getSettingInMillimetersPerSecond("skirt_line_width"));
        }
    }
}


void Wireframe2gcode::finalize()
{
    gcode.finalize(getSettingInMillimetersPerSecond("speed_travel"), getSettingString("machine_end_gcode").c_str());
    for(int e=0; e<getSettingAsCount("machine_extruder_count"); e++)
        gcode.writeTemperatureCommand(e, 0, false);
}
}//namespace cura    
