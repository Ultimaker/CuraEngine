//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_POLYGON_CONNECTOR_H
#define UTILS_POLYGON_CONNECTOR_H

#ifdef BUILD_TESTS
    #include <gtest/gtest_prod.h> //To allow tests to use protected members.
#endif
#include <vector>

#include "IntPoint.h"
#include "polygon.h"
#include "polygonUtils.h"

namespace cura 
{

/*!
 * Class for connecting polygons together into fewer polygons.
 *                          /.                             .
 * \                       /                               .
 *  \                     /                                .
 *   o-------+ . +-------o                                 .
 *           |   |        > bridge which connects the two polygons
 *     o-----+ . +-----o                                   .
 *    /                 \                                  .
 *   /                   \                                 .
 * 
 *  This way two polygons become one.
 * 
 * By repeating such a procedure many polygons can be connected into a single continuous line.
 */
class PolygonConnector
{
#ifdef BUILD_TESTS
    FRIEND_TEST(PolygonConnectorTest, getBridgeTest);
    FRIEND_TEST(PolygonConnectorTest, connectionLengthTest);
#endif
public:
    PolygonConnector(coord_t line_width, coord_t max_dist)
    : line_width(line_width - 5) // a bit less so that consecutive lines which have become connected can still connect to other lines
    //                |                     |                      |
    // ----------o    |      ----------o    |       ----------o,,,,o
    //           |    |  ==>           |    |  ==>
    // -----o    |    |      -----o----o    |       -----o----o----o
    //      |    |    |                     |                      |
    //      |    |    |           o''''o    |            o''''o    |
    //      |    |    |           |    |    |            |    |    |
    , max_dist(max_dist)
    {}

    /*!
     * Add polygons to be connected by a future call to \ref PolygonConnector::connect()
     */
    void add(const Polygons& input)
    {
        for (ConstPolygonRef poly : input)
        {
            input_polygons.push_back(poly);
        }
    }

    /*!
     * Connect as many polygons together as possible and return the resulting polygons.
     * 
     * Algorithm outline:
     * try to connect a polygon to any of the other polygons
     * - if succeeded, add to pool of polygons to connect
     * - if failed, remove from pool and add to the result
     */
    Polygons connect();

protected:
    coord_t line_width; //!< The distance between the line segments which connect two polygons.
    coord_t max_dist; //!< The maximal distance crossed by the connecting segments. Should be more than the \ref line_width in order to accomodate curved polygons.
    std::vector<ConstPolygonPointer> input_polygons; //!< The polygons assembled by calls to \ref PolygonConnector::add()

    /*!
     * Line segment to connect two polygons
     * A bridge consists of two such connections.
     */
    struct PolygonConnection
    {
        ClosestPolygonPoint from; //!< from location in the source polygon
        ClosestPolygonPoint to; //!< to location in the destination polygon

        PolygonConnection(ClosestPolygonPoint from, ClosestPolygonPoint to)
        : from(from)
        , to(to)
        {}

        coord_t getDistance2()
        {
            return vSize2(to.p() - from.p());
        }
    };
    /*!
     * Bridge to connect two polygons twice in order to make it into one polygon.
     * A bridge consists of two connections.
     *     -----o-----o-----
     *          ^     ^
     *        a ^     ^ b      --> connection a is always the left one
     *          ^     ^   --> direction of the two connections themselves.
     *     -----o-----o----
     * 
     * The resulting polygon will travel along the edges in a direction different from each other.
     */
    struct PolygonBridge
    {
        PolygonConnection a; //!< first connection
        PolygonConnection b; //!< second connection
        PolygonBridge(PolygonConnection a, PolygonConnection b)
        : a(a), b(b)
        {}
    };

    std::vector<PolygonBridge> all_bridges; //!< All bridges generated during any call to \ref PolygonConnector::connect(). This is just for keeping scores for debugging etc.

    /*!
     * Connect the two polygons between which the bridge is computed.
     */
    Polygon connectPolygonsAlongBridge(const PolygonBridge& bridge);

    /*!
     * Add the segment from a polygon which is not removed by the bridge.
     * 
     * This function gets called twice in order to connect two polygons together.
     * 
     * Algorithm outline:
     * Add the one vertex from the \p start,
     * then add all vertices from the polygon in between
     * and then add the polygon location from the \p end.
     * 
     * \param[out] result Where to apend the new vertices to
     */
    void addPolygonSegment(const ClosestPolygonPoint& start, const ClosestPolygonPoint& end, PolygonRef result);

    /*!
     * Get the direction between the polygon locations \p from and \p to.
     * This is intended to be the direction of the polygon segment of the short way around the polygon, not the long way around.
     * 
     * The direction is positive for going in the same direction as the vertices are stored.
     * E.g. if \p from is vertex 7 and \p to is vertex 8 then the direction is positive.
     * Otherwise it is negative.
     * 
     * \note \p from and \p to can also be points on the same segment, so their vertex index isn't everything to the algorithm.
     * 
     * \note This function relies on some assumptions about the geometry of polygons you can encounter.
     * It cannot be used as a general purpose function for any two ClosestPolygonPoint
     * For large distances between \p from and \p to the output direction might be 'incorrect'.
     */
    char getPolygonDirection(const ClosestPolygonPoint& from, const ClosestPolygonPoint& to);

    /*!
     * Get the bridge to cross between two polygons.
     * 
     * If no bridge is possible, or if no bridge is found for any reason, then no object is returned.
     * 
     * Algorithm outline:
     * - find the closest first connection between a \p poly and all (other) \p polygons
     * - find the best second connection parallel to that one at a line_width away
     * 
     * if no second connection is found:
     * - find the second connection at half a line width away and 
     * - the first connection at a whole line distance away
     * So as to try and find a bridge which is centered around the initiall found first connection
     */
    std::optional<PolygonBridge> getBridge(ConstPolygonRef poly, std::vector<Polygon>& polygons);

    /*!
     * Get a connection parallel to a given \p first connection at an orthogonal distance line_width from the \p first connection.
     * 
     * From a given \p first connection,
     * walk along both polygons in each direction
     * until we are at a distance of line_width away orthogonally from the line segment of the \p first connection.
     * 
     * For all combinations of such found points:
     * - check whether they are both on the same side of the \p first connection
     * - choose the connection which woukd form the smalles bridge
     */
    std::optional<PolygonConnection> getSecondConnection(PolygonConnection& first);
};


}//namespace cura



#endif//UTILS_POLYGON_CONNECTOR_H
