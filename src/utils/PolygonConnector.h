/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_POLYGON_CONNECTOR_H
#define UTILS_POLYGON_CONNECTOR_H

#include <vector>

#include "optional.h"

#include "polygon.h"
#include "IntPoint.h"
#include "polygonUtils.h"

namespace cura 
{

class PolygonConnectorTest; // fwd decl

/*!
 * Class for connecting polygons together into less polygons
 */
class PolygonConnector
{
    friend class PolygonConnectorTest;
public:
    PolygonConnector(coord_t line_width, coord_t max_dist)
    : line_width(line_width)
    , max_dist(max_dist)
    {}

    void add(const Polygons& input)
    {
        for (ConstPolygonRef poly : input)
        {
            input_polygons.push_back(poly);
        }
    }

    Polygons connect();

protected:
    coord_t line_width;
    coord_t max_dist;
    std::vector<ConstPolygonPointer> input_polygons;

    /*!
     * Line segment to connect two polygons
     */
    struct PolygonConnection
    {
        ClosestPolygonPoint from; //!< from location in the source polygon
        ClosestPolygonPoint to; //!< to location in the destination polygon

        coord_t getDistance2()
        {
            return vSize2(to.p() - from.p());
        }
    };
    /*!
     * bridge to connect two polygons twice in order to make it into one polygon
     *     -----o-----o-----
     *          ^     v
     *        a ^     v b
     *          ^     v
     *     -----o-----o----
     */
    struct PolygonBridge
    {
        PolygonConnection a; //!< first connection
        PolygonConnection b; //!< second connection
    };

    Polygon connect(const PolygonBridge& bridge);
    
    void addPolygonSegment(const ClosestPolygonPoint& start, const ClosestPolygonPoint& end, PolygonRef result);
    char getPolygonDirection(const ClosestPolygonPoint& from, const ClosestPolygonPoint& to);
    std::optional<PolygonBridge> getBridge(ConstPolygonRef poly, std::vector<ConstPolygonPointer>& polygons);
    std::optional<PolygonConnection> getConnection(ConstPolygonRef poly, std::vector<ConstPolygonPointer>& polygons);
    std::optional<PolygonConnection> getSecondConnection(PolygonConnection& first);
};


}//namespace cura



#endif//UTILS_POLYGON_CONNECTOR_H
