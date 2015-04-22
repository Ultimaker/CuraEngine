#ifndef WEAVER_H
#define WEAVER_H

#include "weaveDataStorage.h"
#include "commandSocket.h"
#include "settings.h"

#include "modelFile/modelFile.h" // PrintObject
#include "slicer.h"

#include "utils/polygon.h"

#include "debug.h"

namespace cura
{

/*!
 * Result of finding the closest point to a given within a set of polygons, with extra information on where the point is.
 */
struct ClosestPolygonPoint
{
    Point p; //!< Result location
    PolygonRef poly; //!< Polygon in which the result was found
    int pos; //!< Index to the first point in the polygon of the line segment on which the result was found
    ClosestPolygonPoint(Point p, int pos, PolygonRef poly) :  p(p), poly(poly), pos(pos) {};
    ClosestPolygonPoint(PolygonRef poly) : poly(poly) {};
};

/*!
 * A point within a polygon and the index of which segment in the polygon the point lies on.
 */
struct GivenDistPoint
{
    Point p; //!< Result location
    int pos; //!< Index to the first point in the polygon of the line segment on which the result was found
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
        
        initial_layer_thickness = getSettingInMicrons("layer_height_0");
        connectionHeight = getSettingInMicrons("wireframe_height"); 
        
        extrusionWidth = getSettingInMicrons("wall_line_width_x");
        
        roof_inset = getSettingInMicrons("wireframe_roof_inset"); 
        nozzle_outer_diameter = getSettingInMicrons("machine_nozzle_tip_outer_diameter");      // ___       ___   .
        nozzle_expansion_angle = getSettingInAngleRadians("machine_nozzle_expansion_angle");  //     \_U_/       .
        nozzle_clearance = getSettingInMicrons("wireframe_nozzle_clearance");                // at least line width
        nozzle_top_diameter = tan(nozzle_expansion_angle) * connectionHeight + nozzle_outer_diameter + nozzle_clearance;
    }

    /*!
     * This is the main function for Neith / Weaving / WirePrinting / Webbed printing.
     * Creates a wireframe for the model consisting of horizontal 'flat' parts and connections between consecutive flat parts consisting of UP moves and diagonally DOWN moves.
     * 
     * \param object The object for which to create a wireframe print
     * \param commandSocket the commandSocket
     */
    void weave(PrintObject* object, CommandSocket* commandSocket);
    

private:
    WireFrame wireFrame;
    

/*!
 * Connect two polygons, chainify the second and generate connections from it, supporting on the first polygon.
 * 
 * \param supporting The polygons from which to start the connection
 * \param z0 The height of the \p supporting
 * \param supported The polygons to be supported by the connection from \p supporting to \p supported
 * \param z1 the height of \p supported
 * \param include_last Whether the last full link should be included in the chainified \p parts1 if the last link would be shorter than the normal link size.
 */
    void connect(Polygons& parts0, int z0, Polygons& parts1, int z1, WeaveConnection& result, bool include_last);

/*!
 * Convert polygons, such that they consist of segments/links of uniform size, namely \p nozzle_top_diameter.
 * 
 * \param parts1 The polygons to be chainified
 * \param start_close_to The point from which to start the first link
 * \param include_last governs whether the last segment is smaller or grater than the \p nozzle_top_diameter.
 * If true, the last segment may be smaller.
 */
    void chainify_polygons(Polygons& parts1, Point start_close_to, Polygons& result, bool include_last);
    
/*!
 * The main weaving function.
 * Generate connections between two polygons.
 * The connections consist of zig zags of which the zig is a line from a point in \p supported to the closest point in \p supporting 
 * and the zag is a diagonal line from the same point in \p supported to a point in \p supporting 
 * with a distance equal to Weaver::nozzle_top_diameter from the other point in \p supporting of the zig.
 * 
 * \param supporting The polygons from which to start the connection
 * \param z0 The height of the \p supporting
 * \param supported The polygons to be supported by the connection from \p supporting to \p supported
 * \param z1 the height of \p supported
 * \param result The resulting connection
 */
    void connect_polygons(Polygons& supporting, int z0, Polygons& supported, int z1, WeaveConnection& result);

/*!
 * Creates the roofs and floors which are laid down horizontally.
 */
    void createHorizontalFill(Polygons& lower_top_parts, WeaveLayer& layer, Polygons& layer_above, int z1);
    
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
    void fillRoofs(Polygons& supporting, Polygons& to_be_supported, int direction, int z, WeaveRoof& roofs);
    
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
    void fillFloors(Polygons& supporting, Polygons& to_be_supported, int direction, int z, WeaveRoof& roofs);
    
/*!
 * Filter out parts of connections with small distances; replace by moves.
 * 
 */
    void connections2moves(WeaveRoofPart& inset);
   
/*!
 * Find the point closest to \p from in all polygons in \p polygons.
 */
    static ClosestPolygonPoint findClosest(Point from, Polygons& polygons);
    
/*!
 * Find the point closest to \p from in the polygon \p polygon.
 */
    static ClosestPolygonPoint findClosest(Point from, PolygonRef polygon);
    
/*!
 * Find the point closest to \p from on the line from \p p0 to \p p1
 */
    static Point getClosestOnLine(Point from, Point p0, Point p1);

/*!
 * Find the next point (going along the direction of the polygon) with a distance \p dist from the point \p from within the \p poly.
 * Returns whether another point could be found within the \p poly which can be found before encountering the point at index \p start_idx.
 * The point \p from and the polygon \p poly are assumed to lie on the same plane.
 * 
 * \param start_idx the index of the prev poly point on the poly.
 * \param poly_start_idx The index of the point in the polygon which is to be handled as the start of the polygon. No point further than this point will be the result.
 */
    static bool getNextPointWithDistance(Point from, int64_t dist, const PolygonRef poly, int start_idx, int poly_start_idx, GivenDistPoint& result);


};

}//namespace cura

#endif//WEAVER_H
