#ifndef WEAVER_H
#define WEAVER_H

#include "settings.h"

#include "modelFile/modelFile.h" // PrintObject
#include "slicer.h"

#include "utils/polygon.h"

#include "MACROS.h"

using namespace cura;

ENUM( ExtrusionDirection, UP, DOWN, LEVEL);

struct ExtrudeSegment
{
    Point3 from;
    Point3 to;
    ExtrusionDirection dir;
    ExtrudeSegment(Point3 from, Point3 to, ExtrusionDirection dir) : from(from), to(to), dir(dir) {};
};

struct WireLayerPart
{
    int z0;
    int z1;
    std::vector<ExtrudeSegment> connection;
    int top_index;
    //PolygonRef top; // ref to polygon in layer.top  (! last point in polygon is first point to start printing it!)
    WireLayerPart(int z0_, int z1_, int top_idx) : z0(z0_), z1(z1_), top_index(top_idx) {};
};
struct WireLayer 
{
    std::vector<WireLayerPart> parts;
    Polygons top;
};
struct WireFrame
{
    std::vector<WireLayer> layers;
};

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
public:
    Weaver(SettingsBase* settings_base) : SettingsBase(settings_base)
    {
    };

    
    void weave(PrintObject* object);

private:
    int nozzle_outer_diameter = 600; // ___       ___   .
    int nozzle_head_distance = 4500; //    |     |      .
    int nozzle_expansion_angle = 45; //     \_U_/       .
    int nozzle_top_diameter = 7000; // tan(static_cast<double>(nozzle_expansion_angle)/180.0 * M_PI) * nozzle_head_distance + nozzle_outer_diameter;
    
    void connect(Polygons& parts1, int z1, Polygons& parts2, int z2, WireLayer& result);
    static ClosestPolygonPoint findClosest(Point from, Polygons& polygons);
    static ClosestPolygonPoint findClosest(Point from, PolygonRef polygon);
    static Point getClosestOnLine(Point from, Point p0, Point p1);

//     static bool findFirstPointWithDistance(Point3 from, int64_t dist, PolygonRef poly, int z_polygon, int startingPos, Point3 furtherThen, Point3& result);
//     static bool getPointWithDistance(Point3 c, int64_t d, Point3 a, Point3 b, Point3 furtherThen, Point3& result);
//     static int64_t divide(Point3 a, Point3 b); //!< assumes the two vectors are in the same direction
    
    static bool getNextPointWithDistance(Point from, int64_t dist, const PolygonRef poly, int z_polygon, int start_idx, GivenDistPoint& result);


};

#endif//WEAVER_H
