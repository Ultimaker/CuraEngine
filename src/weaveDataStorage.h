/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef WEAVE_DATA_STORAGE_H
#define WEAVE_DATA_STORAGE_H

#include "utils/NoCopy.h"
#include "utils/intpoint.h"
#include "utils/polygon.h"
#include "mesh.h"
#include "LayerPlan.h"
#include "MeshGroup.h"


namespace cura {


enum class WeaveSegmentType
{
    UP,
    DOWN,
    FLAT,
    MOVE,
    DOWN_AND_FLAT // DOWN_AND_FLAT is for parts of the roof which can either be viewed as flat or as down, since their [to] location is an up move with zero length
};


struct WeaveConnectionSegment
{
    Point3 to;
    WeaveSegmentType segmentType;
    WeaveConnectionSegment(Point3 to, WeaveSegmentType dir) : to(to), segmentType(dir) {};
};

struct PolyLine3
{
    Point3 from;
    std::vector<WeaveConnectionSegment> segments;
};

struct WeaveConnectionPart
{
    PolyLine3 connection;
    int supported_index;//!< index of corresponding supported polygon in WeaveConnection.supported (! last point in polygon is first point to start printing it!)
    WeaveConnectionPart(int top_idx) : supported_index(top_idx) {};
};

struct WeaveConnection
{
    int z0;//!< height of the supporting polygons (of the prev layer, roof inset, etc.)
    int z1;//!< height of the \p supported polygons
    std::vector<WeaveConnectionPart> connections; //!< for each polygon in \p supported the connection.
    Polygons supported; //!< polygons to be supported by connections (from other polygons)
};

// Horizontal Fills:

typedef std::vector<WeaveConnectionSegment> WeaveInsetPart; //!< Polygon with extra information on each point
struct WeaveRoofPart : WeaveConnection
{
    // [supported] is an insets of the roof polygons (or of previous insets of it)
    // [connections] are the connections between two consecutive roof polygon insets
    std::vector<WeaveInsetPart> supported_withMoves; //!< optimized inset polygons, with some parts of the polygons replaced by moves
};

struct WeaveRoof
{
    std::vector<WeaveRoofPart> roof_insets; //!< connections between consecutive insets of the roof polygons
    Polygons roof_outlines; //!< the area within which the horitonal connections are generated
};

// Layers

struct WeaveLayer : WeaveConnection
{
    // [supported] are the outline polygons on the next layer which are (to be) connected,
    //             as well as the polygons supported by roofs (holes and boundaries of roofs)
    // [connections] are the vertical connections
    WeaveRoof roofs; //!< parts which are filled horizontally (both roofs and floors...)
};
struct WireFrame : public NoCopy
{
    MeshGroup* meshgroup;
    WeaveRoof bottom_infill;
    Polygons bottom_outline;
    int z_bottom;
    std::vector<WeaveLayer> layers;
};


}//namespace cura

#endif//WEAVE_DATA_STORAGE_H
