//Copyright (c) 2022 Ultimaker B.V.
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
#include "linearAlg2D.h"

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
 * By repeating such a procedure many polygons can be connected into a single
 * continuous line.
 *
 * This connector can handle ordinary Polygons (which is assumed to print with a
 * fixed, given line width) as well as variable-width paths. However with the
 * paths it will only connect paths that form closed loops. Paths that don't
 * form closed loops will be left unconnected.
 *
 * While this connector can connect Polygons and VariableWidthPaths at the same
 * time, it will never connect them together. This is done to keep the result
 * and the algorithm simpler. Otherwise it would have to convert polygons to
 * paths to make them partially variable width. This is not a use case we need
 * right now, since infill patterns cannot generate a mix of these types.
 *
 * Basic usage of this class is as follows:
 * ``
 * PolygonConnector connector(line_width, max_dist); //Construct first.
 * connector.add(polygons); //Add the polygons and paths you want to connect up.
 * connector.add(paths);
 * Polygons output_polygons; //Prepare some output variables to store results in.
 * VariableWidthPaths output_paths;
 * connector.connect(output_polygons, output_paths);
 * ``
 */
class PolygonConnector
{
#ifdef BUILD_TESTS
    FRIEND_TEST(PolygonConnectorTest, getBridgeTest);
    FRIEND_TEST(PolygonConnectorTest, connectionLengthTest);
#endif
public:
    /*!
     * Create a connector object that can connect polygons.
     *
     * This specifies a few settings for the connector.
     * \param line_width The width at which the polygons will be printed.
     * \param max_dist The maximum length of connections. If polygons can only
     * be connected by creating bridges longer than this distance, they will be
     * left unconnected.
     */
    PolygonConnector(const coord_t line_width, const coord_t max_dist);

    /*!
     * Add polygons to be connected by a future call to \ref PolygonConnector::connect()
     */
    void add(const Polygons& input);

    /*!
     * Add variable-width paths to be connected by a future call to
     * \ref PolygonConnector::connect().
     *
     * Only the paths that form closed loops will be connected to each other.
     * \param input The paths to connect.
     */
    void add(const VariableWidthPaths& input);

    /*!
     * Connect as many polygons together as possible and return the resulting polygons.
     *
     * Algorithm outline:
     * try to connect a polygon to any of the other polygons
     * - if succeeded, add to pool of polygons to connect
     * - if failed, remove from pool and add to the result
     * \param output_polygons Polygons that were connected as much as possible.
     * These are expected to be empty to start with.
     * \param output_paths Paths that were connected as much as possible. These
     * are expected to be empty to start with.
     */
    void connect(Polygons& output_polygons, VariableWidthPaths& output_paths);

protected:
    coord_t line_width; //!< The distance between the line segments which connect two polygons.
    coord_t max_dist; //!< The maximal distance crossed by the connecting segments. Should be more than the \ref line_width in order to accomodate curved polygons.
    std::vector<Polygon> input_polygons; //!< The polygons assembled by calls to \ref PolygonConnector::add.
    std::vector<ExtrusionLine> input_paths; //!< The paths assembled by calls to \ref PolygonConnector::add.

    /*!
     * Line segment to connect two polygons, with all the necessary information
     * to connect them.
     *
     * A bridge consists of two such connections.
     * \tparam Polygonal The type of polygon data to refer to, either Polygon or
     * ExtrusionLine.
     */
    template<typename Polygonal>
    struct PolygonConnection
    {
        /*!
         * The polygon at the source of the connection.
         */
        Polygonal* from_poly;

        /*!
         * The index of the line segment at the source of the connection.
         *
         * This line segment is the one after the vertex with the same index.
         */
        size_t from_segment;

        /*!
         * The precise location of the source of the connection.
         */
        Point from_point;

        /*!
         * The polygon at the destination of the connection.
         */
        Polygonal* to_poly;

        /*!
         * The index of the line segment at the destination of the connection.
         *
         * This line segment is the one after the vertex with the same index.
         */
        size_t to_segment;

        /*!
         * The precise location of the destination of the connection.
         */
        Point to_point;

        /*!
         * Create a new connection.
         * \param from_poly The polygon at the source of the connection.
         * \param from_segment The index of the line segment at the source of
         * the connection.
         * \param from_point The precise location at the source of the
         * connection.
         * \param to_poly The polygon at the destination of the connection.
         * \param to_segment The index of the line segment at the destination of
         * the connection.
         * \param to_point The precise location at the destination of the
         * connection.
         */
        PolygonConnection(Polygonal* from_poly, const size_t from_segment, const Point from_point, Polygonal* to_poly, const size_t to_segment, const Point to_point)
        : from_poly(from_poly)
        , from_segment(from_segment)
        , from_point(from_point)
        , to_poly(to_poly)
        , to_segment(to_segment)
        , to_point(to_point)
        {
        }

        /*!
         * Get the squared length of the connection.
         *
         * The squared length is faster to compute than the real length. Compare
         * it only with the squared maximum distance.
         */
        coord_t getDistance2() const
        {
            return vSize2(from_point - to_point);
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
    template<typename Polygonal>
    struct PolygonBridge
    {
        PolygonConnection<Polygonal> a; //!< first connection
        PolygonConnection<Polygonal> b; //!< second connection
        PolygonBridge(const PolygonConnection<Polygonal>& a, const PolygonConnection<Polygonal>& b)
        : a(a), b(b)
        {}
    };

    /*!
     * Connect a group of polygonal objects - either polygons or lines.
     *
     * This function is generic and will work the same way with either data
     * type. However it will call specialized functions, for instance to get the
     * position of a vertex in the data. This reduces code duplication.
     * \tparam The type of polygonal data to connect.
     * \param to_connect The input polygonals that need to be connected.
     * \return The connected polygonals.
     */
    template<typename Polygonal>
    std::vector<Polygonal> connectGroup(std::vector<Polygonal>& to_connect)
    {
        std::vector<Polygonal> result;
        if(to_connect.empty())
        {
            return result;
        }

        while(!to_connect.empty())
        {
            if(to_connect.size() == 1) //Nothing to connect it to any more.
            {
                result.push_back(to_connect[0]);
                break;
            }
            Polygonal current = std::move(to_connect.back());
            to_connect.pop_back();

            std::optional<PolygonBridge<Polygonal>> bridge = getBridge(current, to_connect);
            if(bridge)
            {
                PolygonRef other_poly(*const_cast<ClipperLib::Path*>(bridge->a.to.poly.operator->())); // const casting a ConstPolygonPointer is difficult!
                other_poly = connectPolygonsAlongBridge(*bridge); //Connect the bridged parts and overwrite the other polygon with it.
                //Don't store the current polygon. It has just been merged into the other one.
            }
            else
            {
                result.push_back(current);
            }
        }
        return result;
    }

    /*!
     * Get the position of a vertex, if the vertex is a point.
     *
     * This overload is simply the identity function. It will return the given
     * vertex. This is a helper function to get the position of generic
     * vertices.
     * \param vertex The vertex to get the position of.
     * \return The position of that vertex.
     */
    Point getPosition(const Point& vertex) const;

    /*!
     * Get the position of a vertex, if the vertex is a junction.
     *
     * This is a helper function to get the position of generic vertices.
     * \param vertex The vertex to get the position of.
     * \return The position of that vertex.
     */
    Point getPosition(const ExtrusionJunction& vertex) const;

    /*!
     * Get the width at a certain vertex.
     *
     * This is the overload that works for fixed-width polygons. They get their
     * width from the width that was given at the constructor.
     */
    coord_t getWidth(const Point& vertex) const;

    /*!
     * Get the width at a certain junction.
     *
     * This is the overload that works for variable-width polygons. The width is
     * stored in the junction then.
     */
    coord_t getWidth(const ExtrusionJunction& vertex) const;

    /*!
     * Get the local width at a certain position along a line segment.
     *
     * If the line segment has a variable width, the local line width will be
     * interpolated between the two endpoints.
     * \param position The position at which to get the line width. This should
     * be (approximately) in between the position of the two vertices.
     * \param a One of the vertices between which to interpolate.
     * \param b The other vertex between which to interpolate.
     */
    template<typename Vertex>
    coord_t interpolateWidth(const Point position, Vertex a, Vertex b) const
    {
        const coord_t total_length = vSize(getPosition(a) - getPosition(b));
        const coord_t position_along_length = vSize(position - getPosition(a));
        return getWidth(b) * position_along_length / total_length + getWidth(a) * (total_length - position_along_length) / total_length;
    }

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
    int16_t getPolygonDirection(const ClosestPolygonPoint& from, const ClosestPolygonPoint& to);

    /*!
     * Find the smallest connection between a polygon and a set of other
     * candidate polygons to connect to.
     */
    template<typename Polygonal>
    std::optional<PolygonConnection<Polygonal>> findConnection(Polygonal from_poly, std::vector<Polygonal>& to_polygons)
    {
        //Optimise for finding the best connection. Track all statistics for the best connection.
        coord_t best_distance = line_width * 0.5; //Allow distance between polygons of up to 1/2 line width, as fudge factor for sharp corners.
        size_t best_poly_index = 0;
        size_t best_from_index = 0;
        size_t best_to_index = 0;
        Point best_from_point;
        Point best_to_point;

        //The smallest connection will be from one of the vertices. So go through all of the vertices to find the closest place where they approach.
        for(size_t poly_index = 0; poly_index < to_polygons.size(); ++poly_index)
        {
            for(size_t to_index = 0; to_index < to_polygons[poly_index].size(); ++to_index)
            {
                const Point to_pos1 =  getPosition(to_polygons[poly_index][to_index]);
                const coord_t to_width1 = getWidth(to_polygons[poly_index][to_index]);
                const Point to_pos2 =  getPosition(to_polygons[poly_index][(to_index + 1) % to_polygons[poly_index].size()]);
                const coord_t to_width2 = getWidth(to_polygons[poly_index][(to_index + 1) % to_polygons[poly_index].size()]);
                const coord_t smallest_to_width = std::min(to_width1, to_width2);

                for(size_t from_index = 0; from_index < from_poly.size(); ++from_index)
                {
                    const Point from_pos1 = getPosition(from_poly[from_index]);
                    const coord_t from_width1 = getWidth(from_poly[from_index]);
                    const Point from_pos2 = getPosition(from_poly[(from_index + 1) % from_poly.size()]);
                    const coord_t from_width2 = getWidth(from_poly[(from_index + 1) % from_poly.size()]);
                    const coord_t smallest_from_width = std::min(from_width1, from_width2);

                    //Try a naive distance first. Faster to compute, but it may estimate the distance too small.
                    coord_t naive_dist = LinearAlg2D::getDistFromLine(from_pos1, to_pos1, to_pos2);
                    if(naive_dist - from_width1 - smallest_to_width < line_width * 0.5)
                    {
                        const Point closest_point = LinearAlg2D::getClosestOnLineSegment(from_pos1, to_pos1, to_pos2);
                        const coord_t width_at_closest = interpolateWidth(closest_point, to_polygons[poly_index][to_index], to_polygons[poly_index][(to_index + 1) % to_polygons[poly_index].size()]);
                        const coord_t distance = vSize(closest_point - from_pos1) - from_width1 - width_at_closest; //Actual, accurate distance to the other polygon.
                        if(distance < best_distance)
                        {
                            best_distance = distance;
                            best_poly_index = poly_index;
                            best_from_index = from_index;
                            best_to_index = to_index;
                            best_from_point = from_pos1;
                            best_to_point = closest_point;
                        }
                    }

                    //Also try the other way around: From the line segment of the from_poly to a vertex in the to_polygons.
                    naive_dist = LinearAlg2D::getDistFromLine(to_pos1, from_pos1, from_pos2);
                    if(naive_dist - smallest_from_width - to_width1 < line_width * 0.5)
                    {
                        const Point closest_point = LinearAlg2D::getClosestOnLineSegment(to_pos1, from_pos1, from_pos2);
                        const coord_t width_at_closest = interpolateWidth(closest_point, from_poly[from_index], from_poly[(from_index + 1) % from_poly.size()]);
                        const coord_t distance = vSize(closest_point - to_pos1) - width_at_closest - to_width1; //Actual, accurate distance.
                        if(distance < best_distance)
                        {
                            best_distance = distance;
                            best_poly_index = poly_index;
                            best_from_index = from_index;
                            best_to_index = to_index;
                            best_from_point = closest_point;
                            best_to_point = to_pos1;
                        }
                    }
                }
            }
        }

        if(best_distance < line_width * 0.5) //The value has been adjusted, so at least one connection was found.
        {
            return PolygonConnection<Polygonal>(&from_poly, best_from_index, best_from_point, &to_polygons[best_poly_index], best_to_index, best_to_point);
        }
        else
        {
            return std::nullopt;
        }
    }

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
