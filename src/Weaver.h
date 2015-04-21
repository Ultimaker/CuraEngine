#ifndef WEAVER_H
#define WEAVER_H

#include "weaveDataStorage.h"
#include "commandSocket.h"
#include "settings.h"

#include "modelFile/modelFile.h" // PrintObject
#include "slicer.h"

#include "utils/polygon.h"

#include "debug.h"

using namespace cura;

/*!
 * Result of finding the closest point to a given within a set of polygons, with extra information on where the point is.
 */
struct ClosestPolygonPoint
{
    Point p;
    PolygonRef poly;
    int pos;
    ClosestPolygonPoint(Point p, int pos, PolygonRef poly) :  p(p), poly(poly), pos(pos) {};
    ClosestPolygonPoint(PolygonRef poly) : poly(poly) {};
};

/*!
 * A point within a polygon and the index of which segment in the polygon the point lies on.
 */
struct GivenDistPoint
{
    Point p;
    int pos;
};

/*!
 * The main weaver / WirePrint / wireframe printing class, which computes the basic paths to be followed.
 */
class Weaver : public SettingsBase
{
    friend class Wireframe2gcode;
private:
    static const int HIGHER_BEND_NO_STRAIGHTEN = 0;
    static const int MOVE_TO_STRAIGHTEN = 1;
    static const int RETRACT_TO_STRAIGHTEN = 2;
    
    int initial_layer_thickness;
    int connectionHeight; 
    int extrusionWidth;
    
    int roof_inset; 
    
    int nozzle_outer_diameter; 
    double nozzle_expansion_angle; 
    int nozzle_clearance; 
    int nozzle_top_diameter;
   
    
public:
    
    Weaver(SettingsBase* settings_base) : SettingsBase(settings_base) 
    {
        
        initial_layer_thickness = getSettingInMicrons("initialLayerThickness");
        connectionHeight = getSettingInMicrons("wireframeConnectionHeight"); 
        
        extrusionWidth = getSettingInMicrons("extrusionWidth");
        
        roof_inset = getSettingInMicrons("wireframeRoofInset"); 
        nozzle_outer_diameter = getSettingInMicrons("machineNozzleTipOuterDiameter");      // ___       ___   .
        nozzle_expansion_angle = getSettingInAngleRadians("machineNozzleExpansionAngle");  //     \_U_/       .
        nozzle_clearance = getSettingInMicrons("wireframeNozzleClearance");                // at least line width
        nozzle_top_diameter = tan(nozzle_expansion_angle) * connectionHeight + nozzle_outer_diameter + nozzle_clearance;
    }

    void weave(PrintObject* object, CommandSocket* commandSocket);
    

private:
    WireFrame wireFrame;
    
    
    void connect(Polygons& parts0, int z0, Polygons& parts1, int z1, WeaveConnection& result, bool include_last);
    
    void chainify_polygons(Polygons& parts1, Point start_close_to, Polygons& result, bool include_last);
    void connect_polygons(Polygons& supporting, int z0, Polygons& supported, int z1, WeaveConnection& result);

    
    void createHorizontalFill(Polygons& lower_top_parts, WeaveLayer& layer, Polygons& layer_above, int z1);
    
    void fillRoofs(Polygons& supporting, Polygons& to_be_supported, int direction, int z, WeaveRoof& roofs);
    void fillFloors(Polygons& supporting, Polygons& to_be_supported, int direction, int z, WeaveRoof& roofs);
    void connections2moves(WeaveRoofPart& inset);
    
    static ClosestPolygonPoint findClosest(Point from, Polygons& polygons);
    static ClosestPolygonPoint findClosest(Point from, PolygonRef polygon);
    static Point getClosestOnLine(Point from, Point p0, Point p1);

    static bool getNextPointWithDistance(Point from, int64_t dist, const PolygonRef poly, int start_idx, int poly_start_idx, GivenDistPoint& result);


};

#endif//WEAVER_H
