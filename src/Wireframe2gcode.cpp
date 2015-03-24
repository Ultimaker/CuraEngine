#include "Wireframe2gcode.h"

#include <cmath> // sqrt
#include <fstream> // debug IO

#include "weaveDataStorage.h"

using namespace cura;





void Wireframe2gcode::writeGCode(CommandSocket* commandSocket, int& maxObjectHeight)
{

    if (commandSocket)
        commandSocket->beginGCode();
    
    if (!gcode.isOpened())
    {
        DEBUG_PRINTLN("gcode not opened!");
        return;
    }
    
    


    /*
    // move to straighten settings
    int speed = 3; //getSettingInt("wireframePrintspeed");
    int bottomSpeed = speed;
    int moveSpeed = 40;
    int upSpeed = speed;
    int downSpeed = speed;
    int flatSpeed = speed;
    */
    /*
    // heighten bend settings
    int speed = 5; 
    int bottomSpeed = speed;
    int moveSpeed = 40;
    int upSpeed = speed;
    int downSpeed = speed;
    int flatSpeed = speed;
    */
    

    

    
    
    
    
    
    
    
    // roofs:
    int roof_inset = connectionHeight; // 45 degrees
    
    
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
            gcode.writeMove(segment_to, speedBottom, extrusion_per_mm_flat);
        }
    }
    
    // bottom:
    writeFill(wireFrame.bottom_insets, 
              [this](Wireframe2gcode& thiss, WireConnectionSegment& segment, WireRoofPart& inset, WireConnectionPart& part, int segment_idx) { gcode.writeMove(segment.to, speedBottom, extrusion_per_mm_connection); }
              , 
              [this](Wireframe2gcode& thiss, Point& p) { gcode.writeMove(p, speedBottom, extrusion_per_mm_flat); }
             );
    
    
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
            WireConnectionPart& part = layer.connections[part_nr];
            
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
            
            gcode.writeComment("TYPE:WALL-OUTER"); // top
            int new_z = initial_layer_thickness + connectionHeight * (layer_nr + 1);
            gcode.setZ(new_z);
            maxObjectHeight = std::max(maxObjectHeight, new_z);
            PolygonRef top_part = layer.supported[part.top_index];
            for (int poly_point = 0; poly_point < top_part.size(); poly_point++)
            {
                gcode.writeMove(top_part[poly_point], speedFlat, extrusion_per_mm_flat);
                gcode.writeDelay(flat_delay);
            }
            
            // roofs:
            std::function<void (Wireframe2gcode& thiss, WireConnectionSegment& segment, WireRoofPart& inset, WireConnectionPart& part, int segment_idx)>
                handle_roof = &Wireframe2gcode::handle_roof_segment;
            writeFill(layer.roof_insets, handle_roof,
                    [this](Wireframe2gcode& thiss, Point& p) { 
                        gcode.writeMove(p, speedFlat, extrusion_per_mm_flat);
                        gcode.writeDelay(flat_delay);
                    });
        }
        
     
        
    }
    
    gcode.setZ(maxObjectHeight);
    
    gcode.writeDelay(0.3);
    
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

    
void Wireframe2gcode::go_down(WireConnectionSegment& segment, WireLayer& layer, WireConnectionPart& part, int segment_idx) 
{ 
    if (go_back_to_last_top)
        gcode.writeMove(segment.from, speedDown, 0);
    if (straight_first_when_going_down <= 0)
    {
        gcode.writeMove(segment.to, speedDown, extrusion_per_mm_connection);
    } else 
    {
        Point3& to = segment.to;
        Point3 from = gcode.getPosition();// segment.from;
        Point3 vec = to - from;
        Point3 in_between = from + vec * straight_first_when_going_down / 100;
//                 Point in_between2D(in_between.x, in_between.y);
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
};


    
void Wireframe2gcode::move_to_straighten(WireConnectionSegment& segment, WireLayer& layer, WireConnectionPart& part, int segment_idx)
{
    gcode.writeMove(segment.to, speedUp, extrusion_per_mm_connection);
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
    
    gcode.writeMove(current_pos - next_dir, speedUp, 0);
    gcode.writeDelay(top_delay);
    gcode.writeMove(current_pos + next_dir_2D, speedUp, 0);
};
void Wireframe2gcode::retract_to_straighten(WireConnectionSegment& segment)
{
    
    RetractionConfig retraction_config;
    retraction_config.amount = 500; //INT2MM(getSettingInt("retractionAmount"))
    retraction_config.primeAmount = 0;//INT2MM(getSettingInt("retractionPrime
    retraction_config.speed = 20; // 40;
    retraction_config.primeSpeed = 15; // 30;
    retraction_config.zHop = 0; //getSettingInt("retractionZHop");

    double top_retract_pause = 2.0;
    int retract_hop_dist = 1000;
    bool after_retract_hop = false;
    bool go_horizontal_first = true;
    bool lower_retract_start = true;
    
    
    Point3& to = segment.to;
    if (lower_retract_start)
    {
        Point3 vec = to - segment.from;
        Point3 lowering = vec * retract_hop_dist / 2 / vec.vSize();
        Point3 lower = to - lowering;
        gcode.writeMove(lower, speedUp, extrusion_per_mm_connection);
        gcode.writeRetraction(&retraction_config, true);
        gcode.writeMove(to + lowering, speedUp, 0);
        gcode.writeDelay(top_retract_pause);
        if (after_retract_hop)
            gcode.writeMove(to + Point3(0, 0, retract_hop_dist), speedFlat, 0);
        
    } else 
    {
        gcode.writeMove(to, speedUp, extrusion_per_mm_connection);
        gcode.writeRetraction(&retraction_config, true);
        gcode.writeMove(to + Point3(0, 0, retract_hop_dist), speedFlat, 0);
        gcode.writeDelay(top_retract_pause);
        if (after_retract_hop)    
            gcode.writeMove(to + Point3(0, 0, retract_hop_dist*3), speedFlat, 0);
        }
};

void Wireframe2gcode::higher_bend(WireConnectionSegment& segment, WireLayer& layer, WireConnectionPart& part, int segment_idx)
{
    Point3 to = segment.to + Point3(0, 0, fall_down);
    Point3 vector = segment.to - segment.from;
    Point3 dir = vector * drag_along / vector.vSize();
    
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
    int64_t next_dir_2D_size = vSize(next_dir_2D);
    if (next_dir_2D_size > 0)
        next_dir_2D = next_dir_2D * drag_along / next_dir_2D_size;
    Point3 next_dir (next_dir_2D.X, next_dir_2D.Y, 0);
    
    gcode.writeMove(to - next_dir + dir, speedUp, extrusion_per_mm_connection);
};
void Wireframe2gcode::handle_segment(WireConnectionSegment& segment, WireLayer& layer, WireConnectionPart& part, int segment_idx) 
{
    if (strategy == STRATEGY_KNOT)
    {
        if (segment.dir == ExtrusionDirection::UP)
        {
            move_to_straighten(segment, layer, part, segment_idx);
        } else 
            go_down(segment, layer, part, segment_idx);
    } else if (strategy == STRATEGY_RETRACT)
    {
        if (segment.dir == ExtrusionDirection::UP)
        {
            retract_to_straighten(segment);
        } else 
            go_down(segment, layer, part, segment_idx);   
    } else if (strategy == STRATEGY_COMPENSATE)
    {
        if (segment.dir == ExtrusionDirection::UP)
        {
            higher_bend(segment, layer, part, segment_idx);
        } else 
            go_down(segment, layer, part, segment_idx);   
    }

};




void Wireframe2gcode::handle_roof_segment(WireConnectionSegment& segment, WireRoofPart& inset, WireConnectionPart& part, int segment_idx)
{
    if (segment.dir == ExtrusionDirection::UP)
    {
        Point3 to = segment.to + Point3(0, 0, roof_fall_down);
        
        Point3 vector = segment.to - segment.from;
        Point3 dir = vector * roof_drag_along / vector.vSize();
        
        Point3 next_vector;
        if (segment_idx + 1 < part.connection.size())
        {
            WireConnectionSegment& next_segment = part.connection[segment_idx+1];
            next_vector = next_segment.to - next_segment.from;
        } else
        {
            next_vector = (segment.to - inset.supported[part.top_index][0]) * -1;
        }
        Point next_dir_2D(next_vector.x, next_vector.y);
        next_dir_2D = next_dir_2D * roof_drag_along / vSize(next_dir_2D);
        Point3 next_dir (next_dir_2D.X, next_dir_2D.Y, 0);
        
        Point3 detoured = to - next_dir + dir;
        
        gcode.writeMove(detoured, speedUp, extrusion_per_mm_connection);
    } else 
    {
        gcode.writeMove(segment.to, speedUp, extrusion_per_mm_connection);
        gcode.writeDelay(roof_outer_delay);
    }

};



void Wireframe2gcode::writeFill(std::vector<WireRoofPart>& fill_insets
    , std::function<void (Wireframe2gcode& thiss, WireConnectionSegment& segment, WireRoofPart& inset, WireConnectionPart& part, int segment_idx)> connectionHandler
    , std::function<void (Wireframe2gcode& thiss, Point& p)> flatHandler)
{
    
    // bottom:
    gcode.writeComment("TYPE:FILL");
    for (int inset_idx = 0; inset_idx < fill_insets.size(); inset_idx++)
    {
        WireRoofPart& inset = fill_insets[inset_idx];
        
        
        for (int inset_part_nr = 0; inset_part_nr < inset.connections.size(); inset_part_nr++)
        {
            WireConnectionPart& inset_part = inset.connections[inset_part_nr];
            
            gcode.writeComment("TYPE:SUPPORT"); // connection
            if (inset_part.connection.size() == 0) continue;
            gcode.writeMove(inset_part.connection[0].from, moveSpeed, 0);
            for (int segment_idx = 0; segment_idx < inset_part.connection.size(); segment_idx++)
            {
                WireConnectionSegment& segment = inset_part.connection[segment_idx];
                connectionHandler(*this, segment, inset, inset_part, segment_idx);
            }
            
            gcode.writeComment("TYPE:WALL-INNER"); // top
            PolygonRef inner_part = inset.supported[inset_part.top_index];
            for (int poly_point = 0; poly_point < inner_part.size(); poly_point++)
            {
                flatHandler(*this, inner_part[poly_point]);
            }
        }
    }
}





Wireframe2gcode::Wireframe2gcode(Weaver& weaver, GCodeExport& gcode, SettingsBase* settings_base) 
: SettingsBase(settings_base) 
, gcode(gcode)
{
    wireFrame = weaver.wireFrame;
    initial_layer_thickness = getSettingInt("initialLayerThickness");
    connectionHeight = getSettingInt("wireframeConnectionHeight"); 
    roof_inset = getSettingInt("wireframeRoofInset"); 
    
    filament_diameter = getSettingInt("filamentDiameter");
    extrusionWidth = getSettingInt("extrusionWidth");
    
    flowConnection = getSettingInt("wireframeFlowConnection");
    flowFlat = getSettingInt("wireframeFlowFlat");
    
    
    
    
    double filament_area = /* M_PI * */ (INT2MM(filament_diameter) / 2.0) * (INT2MM(filament_diameter) / 2.0);
    double lineArea = /* M_PI * */ (INT2MM(extrusionWidth) / 2.0) * (INT2MM(extrusionWidth) / 2.0);
    
    extrusion_per_mm_connection = lineArea / filament_area * double(flowConnection) / 100.0;
    extrusion_per_mm_flat = lineArea / filament_area * double(flowFlat) / 100.0;
    
    nozzle_outer_diameter = getSettingInt("machineNozzleTipOuterDiameter"); // ___       ___   .
    nozzle_head_distance = getSettingInt("machineNozzleHeadDistance");      //    |     |      .
    nozzle_expansion_angle = getSettingInt("machineNozzleExpansionAngle");  //     \_U_/       .
    nozzle_clearance = getSettingInt("wireframeNozzleClearance");    // at least line width
    nozzle_top_diameter = tan(static_cast<double>(nozzle_expansion_angle)/180.0 * M_PI) * connectionHeight + nozzle_outer_diameter + nozzle_clearance;
    
    
    moveSpeed = 40;
    speedBottom =  getSettingInt("wireframePrintspeedBottom");
    speedUp = getSettingInt("wireframePrintspeedUp");
    speedDown = getSettingInt("wireframePrintspeedDown");
    speedFlat = getSettingInt("wireframePrintspeedFlat");

    
    
    
    
    
    flat_delay = getSettingInt("wireframeFlatDelay")/100.0;
    bottom_delay = getSettingInt("wireframeBottomDelay")/100.0;
    top_delay = getSettingInt("wireframeTopDelay")/100.0;
    
    up_dist_half_speed = getSettingInt("wireframeUpDistHalfSpeed");
    
    
    top_jump_dist = getSettingInt("wireframeTopJump");
    
    
    
    
    fall_down = getSettingInt("wireframeFallDown");
    drag_along = getSettingInt("wireframeDragAlong");
    
    
    
    strategy = getSettingInt("wireframeStrategy"); //  HIGHER_BEND_NO_STRAIGHTEN; // RETRACT_TO_STRAIGHTEN; // MOVE_TO_STRAIGHTEN; // 
    
    
    go_back_to_last_top = false;
    straight_first_when_going_down = getSettingInt("wireframeStraightBeforeDown"); // %
    
    
    
    roof_fall_down = getSettingInt("wireframeRoofFallDown");
    roof_drag_along = getSettingInt("wireframeRoofDragAlong");
    roof_outer_delay = getSettingInt("wireframeRoofOuterDelay")/100.0;
    
}


    