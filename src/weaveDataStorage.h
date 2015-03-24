/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef WEAVE_DATA_STORAGE_H
#define WEAVE_DATA_STORAGE_H

#include "utils/intpoint.h"
#include "utils/polygon.h"
#include "mesh.h"
#include "gcodePlanner.h"

#include "MACROS.h"

namespace cura {

    
ENUM( ExtrusionDirection, UP, DOWN);

struct WireConnectionSegment
{
    Point3 from;
    Point3 to;
    ExtrusionDirection dir;
    WireConnectionSegment(Point3 from, Point3 to, ExtrusionDirection dir) : from(from), to(to), dir(dir) {};
};

struct WireConnectionPart
{
    std::vector<WireConnectionSegment> connection;
    int top_index;// index of corresponding top polygon in layer.top  (! last point in polygon is first point to start printing it!)
    WireConnectionPart(int top_idx) : top_index(top_idx) {};
};

struct WireConnection
{
    int z0;//!< height of the supporting polygons (of the prev layer, roof inset, etc.)
    int z1;//!< height of the [supported] polygons
    std::vector<WireConnectionPart> connections; //!< for each polygon in [supported] the connection // \\ // \\ // \\ // \\.
    Polygons supported; //!< polygons to be supported by connections (from other polygons)
};
struct WireRoofPart : WireConnection
{
    // [supported] is an insets of the roof polygons (or of previous insets of it)
    // [connections] are the connections between two consecutive roof polygon insets
};

struct WireLayer : WireConnection
{
    // [supported] are the polygons on the next layer which are (to be) connected
    // [connections] are the vertical connections
    std::vector<WireRoofPart> roof_insets; //!< connections between consecutive insets of the roof polygons
};
struct WireFrame
{
    Polygons bottom;
    std::vector<WireRoofPart> bottom_insets; //!< connections between consecutive insets of the bottom polygons
    std::vector<WireLayer> layers;
};
    
}//namespace cura

#endif//WEAVE_DATA_STORAGE_H
