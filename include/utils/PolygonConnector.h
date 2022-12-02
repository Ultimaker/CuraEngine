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
 * While this connector can connect Polygons and VariableWidthLines at the same
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
 * VariableWidthLines output_paths;
 * connector.connect(output_polygons, output_paths);
 * ``
 */
class PolygonConnector
{
#ifdef BUILD_TESTS
    FRIEND_TEST(PolygonConnectorTest, getBridgeNestedSquares);
    FRIEND_TEST(PolygonConnectorTest, getBridgeAdjacentSquares);
    FRIEND_TEST(PolygonConnectorTest, getBridgeClosest);
    FRIEND_TEST(PolygonConnectorTest, getBridgeTooFar);
    FRIEND_TEST(PolygonConnectorTest, getBridgeTooNarrow);
#endif
public:
    /*!
     * Create a connector object that can connect polygons.
     *
     * This specifies a few settings for the connector.
     * \param line_width The width at which the polygons will be printed.
     */
    PolygonConnector(const coord_t line_width);

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
    void add(const std::vector<VariableWidthLines>& input);

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
    void connect(Polygons& output_polygons, std::vector<VariableWidthLines>& output_paths);

protected:
    coord_t line_width; //!< The distance between the line segments which connect two polygons.
    std::vector<Polygon> input_polygons; //!< The polygons assembled by calls to \ref PolygonConnector::add.
    std::vector<ExtrusionLine> input_paths; //!< The paths assembled by calls to \ref PolygonConnector::add.

    constexpr static Ratio max_gap = 0.5; //!< The maximum allowed gap between lines that get connected, in multiples of the local line width. Allows connections inside corners where the endpoints are slightly apart.

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
     * Connect a group of polygonal objects - either polygons or paths.
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
        while(!to_connect.empty())
        {
            if(to_connect.size() == 1) //Nothing to connect it to any more.
            {
                result.push_back(to_connect[0]);
                break;
            }
            Polygonal current = std::move(to_connect.back());
            to_connect.pop_back();

            if(!isClosed(current)) //Only bridge closed contours.
            {
                result.push_back(current);
                continue;
            }
            std::optional<PolygonBridge<Polygonal>> bridge = getBridge(current, to_connect);
            if(bridge)
            {
                connectPolygonsAlongBridge(*bridge, *bridge->a.to_poly); //Connect the polygons, and store the result in the to_poly.
                //Don't store the current polygon. It has just been merged into the other one.
            }
            else //Can't connect this to anything. Leave it as-is.
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
     * \param vertex The vertex to get the width of.
     * \return The line width of the polygon.
     */
    coord_t getWidth(const Point& vertex) const;

    /*!
     * Get the width at a certain junction.
     *
     * This is the overload that works for variable-width polygons. The width is
     * stored in the junction then.
     * \param vertex The vertex to get the width of.
     * \return The line width at that vertex.
     */
    coord_t getWidth(const ExtrusionJunction& vertex) const;

    /*!
     * Add a vertex at the end of the polygonal object.
     *
     * This is the overload for fixed-width polygons. The width will be ignored.
     * \param polygonal The polygon to add a vertex to.
     * \param position The position of the vertex to add.
     * \param width The width of the vertex to add, ignored in this overload.
     */
    void addVertex(Polygon& polygonal, const Point& position, const coord_t width) const;

    /*!
     * Add a vertex at the end of the polygonal object.
     *
     * This is the overload for fixed-width polygons.
     * \param polygonal The polygon to add a vertex to.
     * \param vertex The vertex to add.
     */
    void addVertex(Polygon& polygonal, const Point& vertex) const;

    /*!
     * Add a vertex at the end of the polygonal object.
     *
     * This is the overload for variable-width paths.
     * \param polygonal The variable-width path to add a vertex to.
     * \param position The position of the vertex to add.
     * \param width The width of the vertex to add.
     */
    void addVertex(ExtrusionLine& polygonal, const Point& position, const coord_t width) const;

    /*!
     * Add a vertex at the end of the polygonal object.
     *
     * This is the overload for variable-width paths.
     * \param polygonal The variable-width path to add a vertex to.
     * \param vertex The vertex to add.
     */
    void addVertex(ExtrusionLine& polygonal, const ExtrusionJunction& vertex) const;

    /*!
     * Tests whether this is a closed polygonal object, rather than a polyline.
     *
     * Polygons are always closed, so this overload will always return true.
     * \param polygonal The polygonal object to check.
     * \return ``true``, indicating that this polygon is closed.
     */
    bool isClosed(Polygon& polygonal) const;

    /*!
     * Tests whether this is a closed polygonal object, rather than a polyline.
     *
     * If the endpoints of the extrusion line meet, it is a closed shape. If
     * not, it is open.
     * \param polygonal The polygonal object to check.
     * \return ``true`` if that shape is closed, or ``false`` otherwise.
     */
    bool isClosed(ExtrusionLine& polygonal) const;

    /*!
     * Construct an empty polygonal object, without any vertices.
     *
     * This is to be template-specialized for every type. The default
     * constructor for some types creates some invalid data.
     * \return An empty polygonal object.
     */
    template<typename Polygonal>
    Polygonal createEmpty() const
    {
        return Polygonal();
    }

    /*!
     * Get the amount of space in between the polygons at the given connection.
     *
     * The space is the length of the connection, minus the width of the
     * lines at the two endpoints.
     * \param connection The connection to calculate the space of.
     */
    template<typename Polygonal>
    coord_t getSpace(const PolygonConnection<Polygonal>& connection) const
    {
        const coord_t from_width = interpolateWidth(connection.from_point, (*connection.from_poly)[connection.from_segment], (*connection.from_poly)[(connection.from_segment + 1) % connection.from_poly->size()]);
        const coord_t to_width = interpolateWidth(connection.to_point, (*connection.to_poly)[connection.to_segment], (*connection.to_poly)[(connection.to_segment + 1) % connection.to_poly->size()]);
        return vSize(connection.to_point - connection.from_point) - from_width / 2 - to_width / 2;
    }

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
        if(total_length == 0) //Prevent division by 0 when the vertices are on top of each other.
        {
            return getWidth(a); //Just return one of them. They are on top of each other anyway.
        }
        const coord_t position_along_length = vSize(position - getPosition(a));
        return round_divide(getWidth(b) * position_along_length, total_length) + round_divide(getWidth(a) * (total_length - position_along_length), total_length);
    }

    /*!
     * Find the smallest connection between a polygon and a set of other
     * candidate polygons to connect to.
     */
    template<typename Polygonal>
    std::optional<PolygonBridge<Polygonal>> findConnection(Polygonal& from_poly, std::vector<Polygonal>& to_polygons)
    {
        //Optimise for finding the best connection.
        coord_t best_distance = line_width * max_gap; //Allow up to the max_gap.
        std::optional<PolygonConnection<Polygonal>> best_connection;
        std::optional<PolygonConnection<Polygonal>> best_second_connection;

        //The smallest connection will be from one of the vertices. So go through all of the vertices to find the closest place where they approach.
        for(size_t poly_index = 0; poly_index < to_polygons.size(); ++poly_index)
        {
            if(!isClosed(to_polygons[poly_index]))
            {
                continue;
            }
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
                    if(naive_dist - from_width1 - smallest_to_width < line_width * max_gap)
                    {
                        const Point closest_point = LinearAlg2D::getClosestOnLineSegment(from_pos1, to_pos1, to_pos2);
                        if(closest_point == to_pos2) //The last endpoint of a vertex is considered to be part of the next segment. Let that one handle it.
                        {
                            continue;
                        }
                        const coord_t width_at_closest = interpolateWidth(closest_point, to_polygons[poly_index][to_index], to_polygons[poly_index][(to_index + 1) % to_polygons[poly_index].size()]);
                        const coord_t distance = vSize(closest_point - from_pos1) - from_width1 - width_at_closest; //Actual, accurate distance to the other polygon.
                        if(distance < best_distance)
                        {
                            PolygonConnection<Polygonal> first_connection = PolygonConnection<Polygonal>(&from_poly, from_index, from_pos1, &to_polygons[poly_index], to_index, closest_point);
                            std::optional<PolygonConnection<Polygonal>> second_connection = getSecondConnection(first_connection, (width_at_closest + from_width1) / 2);
                            if(second_connection) //Second connection is also valid.
                            {
                                best_distance = distance;
                                best_connection = first_connection;
                                best_second_connection = second_connection;
                            }
                        }
                    }

                    //Also try the other way around: From the line segment of the from_poly to a vertex in the to_polygons.
                    naive_dist = LinearAlg2D::getDistFromLine(to_pos1, from_pos1, from_pos2);
                    if(naive_dist - smallest_from_width - to_width1 < line_width * max_gap)
                    {
                        const Point closest_point = LinearAlg2D::getClosestOnLineSegment(to_pos1, from_pos1, from_pos2);
                        if(closest_point == from_pos2) //The last endpoint of a vertex is considered to be part of the next segment. Let that one handle it.
                        {
                            continue;
                        }
                        const coord_t width_at_closest = interpolateWidth(closest_point, from_poly[from_index], from_poly[(from_index + 1) % from_poly.size()]);
                        const coord_t distance = vSize(closest_point - to_pos1) - width_at_closest - to_width1; //Actual, accurate distance.
                        if(distance < best_distance)
                        {
                            PolygonConnection<Polygonal> first_connection = PolygonConnection<Polygonal>(&from_poly, from_index, closest_point, &to_polygons[poly_index], to_index, to_pos1);
                            std::optional<PolygonConnection<Polygonal>> second_connection = getSecondConnection(first_connection, (to_width1 + width_at_closest) / 2);
                            if(second_connection) //Second connection is also valid.
                            {
                                best_distance = distance;
                                best_connection = first_connection;
                                best_second_connection = second_connection;
                            }
                        }
                    }
                }
            }
        }

        if(best_connection)
        {
            return PolygonBridge<Polygonal>(*best_connection, *best_second_connection);
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
    template<typename Polygonal>
    std::optional<PolygonBridge<Polygonal>> getBridge(Polygonal& from_poly, std::vector<Polygonal>& to_polygons)
    {
        std::optional<PolygonBridge<Polygonal>> connection = findConnection(from_poly, to_polygons);
        if(!connection) //We didn't find a connection. No bridge.
        {
            return std::nullopt;
        }

        //Ensure that B is always the right connection and A the left.
        if(LinearAlg2D::pointIsLeftOfLine(connection->b.from_point, connection->a.from_point, connection->a.to_point) > 0)
        {
            std::swap(connection->a, connection->b);
        }
        return connection;
    }

    /*!
     * Walk along a polygon to find the first point that is exactly ``distance``
     * away from a given line.
     *
     * The resulting point does not have to be exactly on a vertex. Most likely
     * it will be on a line segment.
     * \param poly The polygonal shape along which to walk.
     * \param start_index The vertex at which to start looking. This vertex
     * should be on the wrong side of the line.
     * \param distance The distance from the line at which the resulting point
     * should be.
     * \param line_a The line passes through this point.
     * \param line_b The line also passes through this point.
     * \param direction Use +1 to iterate in the forward direction through the
     * polygon, or -1 to iterate backwards.
     * \return If there is a point that is the correct distance from the line,
     * the first such point is returned, and the segment index that it's on. If
     * the polygon is entirely close to the line, returns
     * ``std::nullopt``.
     */
    template<typename Polygonal>
    std::optional<std::pair<Point, size_t>> walkUntilDistanceFromLine(const Polygonal& poly, const size_t start_index, const coord_t distance, const Point& line_a, const Point& line_b, const short direction)
    {
        const size_t poly_size = poly.size();
        const coord_t line_magnitude = vSize(line_b - line_a); //Pre-compute, used for line distance calculation.
        if(line_magnitude == 0)
        {
            return std::nullopt; //Line doesn't have a direction, so we can't be on any one side of it.
        }

        for(size_t index = (start_index + direction + poly_size) % poly_size; index != start_index; index = (index + direction + poly_size) % poly_size)
        {
            const Point vertex_pos = getPosition(poly[index]);
            const coord_t vertex_distance = cross(line_a - line_b, line_a - vertex_pos) / line_magnitude; //Signed distance!
            if(std::abs(vertex_distance) >= distance) //Further away from the line than the threshold.
            {
                //Interpolate over that last line segment to find the point at exactly the right distance.
                const size_t previous_index = (index - direction + poly_size) % poly_size;
                const Point previous_pos = getPosition(poly[previous_index]);
                const coord_t previous_distance = cross(line_a - line_b, line_a - previous_pos) / line_magnitude;
                if(previous_distance == vertex_distance) //0-length line segment, or parallel to line.
                {
                    continue;
                }
                const double interpolation_pos = double(distance - previous_distance) / (vertex_distance - previous_distance);
                const double interpolation_neg = double(-distance - previous_distance) / (vertex_distance - previous_distance);
                double interpolation;
                if(interpolation_pos >= 0 && interpolation_pos < 1)
                {
                    interpolation = interpolation_pos;
                }
                else if(interpolation_neg >= 0 && interpolation_neg < 1)
                {
                    interpolation = interpolation_neg;
                }
                else
                {
                    continue;
                }
                const Point interpolated_point = previous_pos + (vertex_pos - previous_pos) * interpolation;
                return std::make_pair(interpolated_point, (direction == +1) ? previous_index : index); //Choose the "earlier" index of the two, regardless of direction.
            }
        }
        return std::nullopt; //None of the vertices were far enough away from the line.
    }

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
    template<typename Polygonal>
    std::optional<PolygonConnection<Polygonal>> getSecondConnection(PolygonConnection<Polygonal>& first, const coord_t adjacent_distance)
    {
        std::optional<PolygonConnection<Polygonal>> result = std::nullopt;
        coord_t best_connection_length = std::numeric_limits<coord_t>::max();

        //Find the four intersections, on both sides of the initial connection, and on both polygons.
        std::optional<std::pair<Point, size_t>> from_forward_intersection = walkUntilDistanceFromLine(*first.from_poly, first.from_segment, adjacent_distance, first.from_point, first.to_point, +1);
        std::optional<std::pair<Point, size_t>> from_backward_intersection = walkUntilDistanceFromLine(*first.from_poly, first.from_segment, adjacent_distance, first.from_point, first.to_point, -1);
        std::optional<std::pair<Point, size_t>> to_forward_intersection = walkUntilDistanceFromLine(*first.to_poly, first.to_segment, adjacent_distance, first.from_point, first.to_point, +1);
        std::optional<std::pair<Point, size_t>> to_backward_intersection = walkUntilDistanceFromLine(*first.to_poly, first.to_segment, adjacent_distance, first.from_point, first.to_point, -1);

        for(const std::optional<std::pair<Point, size_t>>& from_intersection : {from_forward_intersection, from_backward_intersection})
        {
            if(!from_intersection)
            {
                continue;
            }
            //Find the shortest of the connections in the to_poly.
            const bool original_side = LinearAlg2D::pointIsLeftOfLine(first.to_point, first.from_point, from_intersection->first) > 0;
            for(const std::optional<std::pair<Point, size_t>>& to_intersection : {to_forward_intersection, to_backward_intersection})
            {
                if(!to_intersection)
                {
                    continue;
                }
                const bool current_side = LinearAlg2D::pointIsLeftOfLine(to_intersection->first, first.from_point, from_intersection->first) > 0;
                if (original_side != current_side)
                {
                    continue;
                }
                PolygonConnection<Polygonal> connection(first.from_poly, from_intersection->second, from_intersection->first, first.to_poly, to_intersection->second, to_intersection->first);
                const coord_t connection_length = getSpace(connection);
                if(connection_length < max_gap * line_width && connection_length < best_connection_length) //Connection is allowed.
                {
                    result = connection;
                    best_connection_length = connection_length;
                }
            }
        }
        return result;
    }

    template<typename Polygonal>
    void connectPolygonsAlongBridge(const PolygonBridge<Polygonal>& bridge, Polygonal& result)
    {
        //We'll traverse the following path:
        //
        // <<<<<<X......X<<<<<<< to_poly
        //       ^      v
        //       ^      v
        //       ^ a  b v bridge
        //       ^      v
        // >>>>>>X......X>>>>>>> from_poly
        //
        //To do this, from_poly and to_poly might need to be traversed in reverse order. This function figures all of that out.

        Polygonal ret = createEmpty<Polygonal>(); //Create a temporary result that we'll move into the result.

        const size_t from_size = bridge.b.from_poly->size();
        //Add the from-endpoint of B.
        const coord_t b_from_width = interpolateWidth(bridge.b.from_point, (*bridge.b.from_poly)[bridge.b.from_segment], (*bridge.b.from_poly)[(bridge.b.from_segment + 1) % from_size]);
        addVertex(ret, bridge.b.from_point, b_from_width);

        //Add the from-polygonal from B to A.
        short forwards;
        if(bridge.a.from_segment == bridge.b.from_segment) //If we start and end on the same segment, iterate in the direction from A to B.
        {
            const Point vertex = getPosition((*bridge.b.from_poly)[bridge.b.from_segment]); //Same vertex for A and B.
            const Point next_vertex = getPosition((*bridge.b.from_poly)[(bridge.b.from_segment + 1) % from_size]);
            const Point direction = next_vertex - vertex; //Direction we'd go into when forward iterating.
            const Point a_to_b = bridge.b.from_point - bridge.a.from_point;
            forwards = vSize2(direction - a_to_b) < vSize2(-direction - a_to_b);
        }
        else
        {
            //If not the same segment, traverse in whichever direction is the long way around.
            forwards = ((bridge.b.from_segment + from_size - bridge.a.from_segment) % from_size) < ((bridge.a.from_segment + from_size - bridge.b.from_segment) % from_size);
        }
        size_t first_segment = forwards ? (bridge.b.from_segment + 1) % from_size : (bridge.b.from_segment + from_size) % from_size;
        size_t last_segment = forwards ? bridge.a.from_segment : bridge.a.from_segment;
        if(first_segment == last_segment) last_segment = (last_segment + from_size - 2 * forwards + 1) % from_size;
        size_t i = first_segment;
        do //Since we might start and end on the same segment, do a do_while loop to iterate at least once.
        {
            addVertex(ret, (*bridge.b.from_poly)[i]);
            i = (i + 2 * forwards - 1 + from_size) % from_size;
        }
        while(i != (last_segment + from_size + 2 * forwards - 1) % from_size);

        //Add the from-endpoint of A.
        const coord_t a_from_width = interpolateWidth(bridge.a.from_point, (*bridge.b.from_poly)[bridge.a.from_segment], (*bridge.b.from_poly)[(bridge.a.from_segment + 1) % from_size]);
        addVertex(ret, bridge.a.from_point, a_from_width);

        const size_t to_size = bridge.b.to_poly->size();
        //Add the to-endpoint of A.
        const coord_t a_to_width = interpolateWidth(bridge.a.to_point, (*bridge.a.to_poly)[bridge.a.to_segment], (*bridge.a.to_poly)[(bridge.a.to_segment + 1) % to_size]);
        addVertex(ret, bridge.a.to_point, a_to_width);

        //Add the to_polygonal from A to B.
        if(bridge.a.to_segment == bridge.b.to_segment)
        {
            const Point vertex = getPosition((*bridge.b.to_poly)[bridge.b.to_segment]); //Same vertex for A and B.
            const Point next_vertex = getPosition((*bridge.b.to_poly)[(bridge.b.to_segment + 1) % to_size]);
            const Point direction = next_vertex - vertex;
            const Point a_to_b = bridge.b.to_point - bridge.a.to_point;
            forwards = vSize2(direction - a_to_b) > vSize2(-direction - a_to_b);
        }
        else
        {
            forwards = ((bridge.a.to_segment + to_size - bridge.b.to_segment) % to_size) < ((bridge.b.to_segment + to_size - bridge.a.to_segment) % to_size);
        }
        first_segment = forwards ? (bridge.a.to_segment + 1) % to_size : bridge.a.to_segment;
        size_t end_segment = forwards ? (bridge.b.to_segment + 1) % to_size : bridge.b.to_segment;
        i = first_segment;
        do
        {
            addVertex(ret, (*bridge.b.to_poly)[i]);
            i = (i + 2 * forwards - 1 + to_size) % to_size;
        }
        while(i != end_segment);

        //Add the to-endpoint of B.
        const coord_t b_to_width = interpolateWidth(bridge.b.to_point, (*bridge.b.to_poly)[bridge.b.to_segment], (*bridge.b.to_poly)[(bridge.b.to_segment + 1) % to_size]);
        addVertex(ret, bridge.b.to_point, b_to_width);

        if(getPosition(ret.back()) != getPosition(ret.front()))
        {
            addVertex(ret, getPosition(ret.front()), getWidth(ret.front()));
        }

        result = std::move(ret); //Override the result with the new combined shape.
    }
};


}//namespace cura



#endif//UTILS_POLYGON_CONNECTOR_H
