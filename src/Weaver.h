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
    friend class Wireframe2gcode;
private:
    static const int HIGHER_BEND_NO_STRAIGHTEN = 0;
    static const int MOVE_TO_STRAIGHTEN = 1;
    static const int RETRACT_TO_STRAIGHTEN = 2;
    
    int initial_layer_thickness;
    int connectionHeight; // = getSettingInt("wireframeConnectionHeight"); 
    
    int roof_inset; // = getSettingInt("wireframeRoofInset"); 
    
    int nozzle_outer_diameter; // = getSettingInt("machineNozzleTipOuterDiameter"); // ___       ___   .
    //int nozzle_head_distance;  = getSettingInt("machineNozzleHeadDistance");      //    |     |      .
    int nozzle_expansion_angle; // = getSettingInt("machineNozzleExpansionAngle");  //     \_U_/       .
    int nozzle_clearance; // = getSettingInt("wireframeNozzleClearance");    // at least line width
    int nozzle_top_diameter; // = tan(static_cast<double>(nozzle_expansion_angle)/180.0 * M_PI) * connectionHeight + nozzle_outer_diameter + nozzle_clearance;
   
    
public:
    
    Weaver(SettingsBase* settings_base) : SettingsBase(settings_base) 
    {
        
        initial_layer_thickness = getSettingInt("initialLayerThickness");
        connectionHeight = getSettingInt("wireframeConnectionHeight"); 
        
        roof_inset = getSettingInt("wireframeRoofInset"); 
        nozzle_outer_diameter = getSettingInt("machineNozzleTipOuterDiameter"); // ___       ___   .
        nozzle_expansion_angle = getSettingInt("machineNozzleExpansionAngle");  //     \_U_/       .
        nozzle_clearance = getSettingInt("wireframeNozzleClearance");    // at least line width
        nozzle_top_diameter = tan(static_cast<double>(nozzle_expansion_angle)/180.0 * M_PI) * connectionHeight + nozzle_outer_diameter + nozzle_clearance;
     
        
    };

    
    void weave(PrintObject* object);
    

private:
    WireFrame wireFrame;
    
    
    
    
    static Polygons getOuterPolygons(Polygons& in);
    static void getOuterPolygons(Polygons& in, Polygons& result);
    
    void connect(Polygons& parts0, int z0, Polygons& parts1, int z1, WireConnection& result);
    
    void chainify_polygons(Polygons& parts1, int z, Polygons& top_parts);
    void connect_polygons(Polygons& supporting, int z0, Polygons& supported, int z1, WireConnection& result);

    
    
    template<class WireConnection_>
    void fillHorizontal(Polygons& outlines, int z, std::vector<WireConnection_>& result);
    
    static ClosestPolygonPoint findClosest(Point from, Polygons& polygons);
    static ClosestPolygonPoint findClosest(Point from, PolygonRef polygon);
    static Point getClosestOnLine(Point from, Point p0, Point p1);

    static bool getNextPointWithDistance(Point from, int64_t dist, const PolygonRef poly, int z_polygon, int start_idx, GivenDistPoint& result);


};

#endif//WEAVER_H
