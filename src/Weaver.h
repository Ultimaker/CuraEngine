//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef WEAVER_H
#define WEAVER_H

#include "weaveDataStorage.h"
#include "utils/NoCopy.h"

namespace cura
{

class MeshGroup;
class Polygons;

/*!
 * The main weaver / WirePrint / wireframe printing class, which computes the basic paths to be followed.
 */
class Weaver : public NoCopy
{
    friend class Wireframe2gcode;
private:
    static const int HIGHER_BEND_NO_STRAIGHTEN = 0;
    static const int MOVE_TO_STRAIGHTEN = 1;
    static const int RETRACT_TO_STRAIGHTEN = 2;

public:
    /*!
     * This is the main function for Neith / Weaving / WirePrinting / Webbed printing.
     * Creates a wireframe for the model consisting of horizontal 'flat' parts and connections between consecutive flat parts consisting of UP moves and diagonally DOWN moves.
     * 
     * \param objects The objects for which to create a wireframe print
     */
    void weave(MeshGroup* objects);

private:
    WireFrame wireFrame;
    

/*!
 * Connect two polygons, chainify the second and generate connections from it, supporting on the first polygon.
 * 
 * \param supporting The polygons from which to start the connection
 * \param z0 The height of the \p supporting
 * \param supported The polygons to be supported by the connection from \p supporting to \p supported
 * \param z1 the height of \p supported
 */
    void connect(Polygons& parts0, int z0, Polygons& parts1, int z1, WeaveConnection& result);

/*!
 * Convert polygons, such that they consist of segments/links of uniform size, namely \p nozzle_top_diameter.
 * 
 * \param parts1 The polygons to be chainified
 * \param start_close_to The point from which to start the first link
 */
    void chainify_polygons(Polygons& parts1, Point start_close_to, Polygons& result);
    
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
    void createHorizontalFill(WeaveLayer& layer, Polygons& layer_above);
    
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
   
};

}//namespace cura

#endif//WEAVER_H
