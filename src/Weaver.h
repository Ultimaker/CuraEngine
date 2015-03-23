#ifndef WEAVER_H
#define WEAVER_H

#include "weaveDataStorage.h"
#include "commandSocket.h"
#include "settings.h"

#include "modelFile/modelFile.h" // PrintObject
#include "slicer.h"

#include "utils/polygon.h"

#include "MACROS.h"

using namespace cura;


struct ClosestPolygonPoint
{
    Point p;
    PolygonRef poly;
    int pos;
    ClosestPolygonPoint(Point p, int pos, PolygonRef poly) :  p(p), pos(pos), poly(poly) {};
    ClosestPolygonPoint(PolygonRef poly) : poly(poly) {};
};

struct GivenDistPoint
{
    Point p;
    int pos;
};

class Weaver : public SettingsBase
{
private:
    static const int HIGHER_BEND_NO_STRAIGHTEN = 0;
    static const int MOVE_TO_STRAIGHTEN = 1;
    static const int RETRACT_TO_STRAIGHTEN = 2;
    
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
    
    
    
public:
    
    Weaver(SettingsBase* settings_base) : SettingsBase(settings_base) 
    {
        
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
        
    };

    
    void weave(PrintObject* object);
    
    void writeGCode(GCodeExport& gcode, CommandSocket* commandSocket, int& maxObjectHeight);


private:
    WireFrame wireFrame;
    
    
    
    
    static Polygons getOuterPolygons(Polygons& in);
    static void getOuterPolygons(Polygons& in, Polygons& result);
    
    void connect(Polygons& parts0, int z0, Polygons& parts1, int z1, WireConnection& result);
    
    template<class WireConnection_>
    void fillHorizontal(Polygons& outlines, int z, std::vector<WireConnection_>& result);
    
    static ClosestPolygonPoint findClosest(Point from, Polygons& polygons);
    static ClosestPolygonPoint findClosest(Point from, PolygonRef polygon);
    static Point getClosestOnLine(Point from, Point p0, Point p1);

    static bool getNextPointWithDistance(Point from, int64_t dist, const PolygonRef poly, int z_polygon, int start_idx, GivenDistPoint& result);


};

#endif//WEAVER_H
