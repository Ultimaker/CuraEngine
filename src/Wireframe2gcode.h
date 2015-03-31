#ifndef WIREFRAME2GCODE_H
#define WIREFRAME2GCODE_H


#include <functional> // passing function pointer or lambda as argument to a function

#include "weaveDataStorage.h"
#include "commandSocket.h"
#include "settings.h"

#include "modelFile/modelFile.h" // PrintObject
#include "slicer.h"

#include "utils/polygon.h"
#include "Weaver.h"

#include "MACROS.h"

using namespace cura;

class Wireframe2gcode : public SettingsBase
{
private:
    static const int STRATEGY_COMPENSATE = 0;
    static const int STRATEGY_KNOT = 1;
    static const int STRATEGY_RETRACT = 2;
    
    int initial_layer_thickness;
    int filament_diameter;
    int extrusionWidth;
    int flowConnection;// = getSettingInt("wireframeFlowConnection");
    int flowFlat; // = getSettingInt("wireframeFlowFlat");
    double extrusion_per_mm_connection; // = lineArea / filament_area * double(flowConnection) / 100.0;
    double extrusion_per_mm_flat; // = lineArea / filament_area * double(flowFlat) / 100.0;
    int nozzle_outer_diameter; // = getSettingInt("machineNozzleTipOuterDiameter"); // ___       ___   .
    int nozzle_head_distance; // = getSettingInt("machineNozzleHeadDistance");      //    |     |      .
    int nozzle_expansion_angle; // = getSettingInt("machineNozzleExpansionAngle");  //     \_U_/       .
    int nozzle_clearance; // = getSettingInt("wireframeNozzleClearance");    // at least line width
    int nozzle_top_diameter; // = tan(static_cast<double>(nozzle_expansion_angle)/180.0 * M_PI) * connectionHeight + nozzle_outer_diameter + nozzle_clearance;
    int moveSpeed; // = 40;
    int speedBottom; // =  getSettingInt("wireframePrintspeedBottom");
    int speedUp; // = getSettingInt("wireframePrintspeedUp");
    int speedDown; // = getSettingInt("wireframePrintspeedDown");
    int speedFlat; // = getSettingInt("wireframePrintspeedFlat");
    int connectionHeight; // = getSettingInt("wireframeConnectionHeight"); 
    int roof_inset; // = getSettingInt("wireframeRoofInset"); 
    double flat_delay; // = getSettingInt("wireframeFlatDelay")/100.0;
    double bottom_delay; // = getSettingInt("wireframeBottomDelay")/100.0;
    double top_delay; // = getSettingInt("wireframeTopDelay")/100.0;
    int up_dist_half_speed; // = getSettingInt("wireframeUpDistHalfSpeed");
    int top_jump_dist; // = getSettingInt("wireframeTopJump");
    int fall_down; // = getSettingInt("wireframeFallDown");
    int drag_along; // = getSettingInt("wireframeDragAlong");
    int strategy; // = getSettingInt("wireframeStrategy"); //  HIGHER_BEND_NO_STRAIGHTEN; // RETRACT_TO_STRAIGHTEN; // MOVE_TO_STRAIGHTEN; // 
    double go_back_to_last_top; // = false;
    int straight_first_when_going_down; // = getSettingInt("wireframeStraightBeforeDown"); // %
    int roof_fall_down; // = getSettingInt("wireframeRoofFallDown");
    int roof_drag_along; // = getSettingInt("wireframeRoofDragAlong");
    double roof_outer_delay; // = getSettingInt("wireframeRoofOuterDelay")/100.0;
    
    RetractionConfig standard_retraction_config;
    
public:
    GCodeExport& gcode;
    
    Wireframe2gcode(Weaver& weaver, GCodeExport& gcode, SettingsBase* settings_base);
    
    void writeGCode(CommandSocket* commandSocket, int& maxObjectHeight);


private:
    WireFrame wireFrame;
    
    void writeFill(std::vector<WeaveRoofPart>& fill_insets
        , std::function<void (Wireframe2gcode& thiss, WeaveRoofPart& inset, WeaveConnectionPart& part, int segment_idx)> connectionHandler
        , std::function<void (Wireframe2gcode& thiss, WeaveConnectionSegment& p)> flatHandler);
    
    void go_down(WeaveLayer& layer, WeaveConnectionPart& part, int segment_idx) ;
    void strategy_knot(WeaveLayer& layer, WeaveConnectionPart& part, int segment_idx);
    void strategy_retract(WeaveLayer& layer, WeaveConnectionPart& part, int segment_idx);
    void strategy_compensate(WeaveLayer& layer, WeaveConnectionPart& part, int segment_idx);
    void handle_segment(WeaveLayer& layer, WeaveConnectionPart& part, int segment_idx);
    
    void handle_roof_segment(WeaveRoofPart& inset, WeaveConnectionPart& part, int segment_idx);

};

#endif//WIREFRAME2GCODE_H
