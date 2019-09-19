//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_ZIGZAG_CONNECTOR_PROCESSOR_H
#define INFILL_ZIGZAG_CONNECTOR_PROCESSOR_H

#include "../utils/polygon.h" //TODO: We have implementation in this header file!

namespace cura
{

/*!
 * Processor class for processing the connections between lines which makes the infill a zigzag pattern.
 * 
 * During the creation of the infill lines, calls are made to a ZigzagConnectorProcessor so that the zigzag connector segments are created 
 * at the same time as the lines are created.
 *
 * generate lines within the area of [in_outline], at regular intervals of [line_distance]
 *  idea:
 * intersect a regular grid of 'scanlines' with the area inside [in_outline] (see generateLineInfill)
 * zigzag:
 * include pieces of boundary, connecting the lines, forming an accordion like zigzag instead of separate lines    |_|^|_|
 * 
 * we call the areas between two consecutive scanlines a 'scansegment'
 * 
 * algorithm:
 * 1. for each line segment of each polygon:
 *      store the intersections of that line segment with all scanlines in a mapping (vector of vectors) from scanline to intersections
 *      (zigzag): add boundary segments to result
 * 2. for each scanline:
 *      sort the associated intersections 
 *      and connect them using the even-odd rule
 * 
 * zigzag algorithm:
 *  while walking around (each) polygon (1.)
 *  if polygon intersects with even scanline
 *      start boundary segment (add each following segment to the [result])
 *  when polygon intersects with a scanline again
 *      stop boundary segment (stop adding segments to the [result])
 *      if polygon intersects with even scanline again (instead of odd)
 *           dont add the last line segment to the boundary (unless [connected_zigzags])
 * 
 * Note that ZigZag consists of 3 types:
 * - without endpieces
 * - with disconnected endpieces
 * - with connected endpieces
 *  <<extra>> there is also a NoZigzagConnector which creates no zags. It is used for the Line infill pattern
 * 
 *      v   v   zigzag connectors
 *     <--
 *    :___:   :      < scanlines
 *    |   |   |
 *    |   |   |      < infill lines along scanlines
 *    |   |___|
 *    :   :   :
 *         -->       winding order of polygon
 * 
 *        ^ = even scanline
 *  ^            ^ no endpieces
 * 
 * start boundary from even scanline! :D
 * include only a boundary segment if it starts in an even scanline and ends in an odd scanline
 * 
 *          ________
 *   |     |     |  \                      .
 *   |     |     |  |
 *   |_____|     |__/                       .
 * 
 *   ^     ^     ^    scanlines
 *                 ^  connected end piece
 * include a boundary segment also if it starts in an odd scanline and ends odd, 
 * or starts in an even scanline and ends in an even scanline,
 * but not when it starts in an odd and ends in an even scanline (see top left or bottom middle).
 * 
 *          _____
 *   |     |     |  \                     .
 *   |     |     |  |
 *   |_____|     |__/
 * 
 *   ^     ^     ^    scanlines
 *                 ^  disconnected end piece
 * Leave out the last line segment of the boundary polygon: from a vertex to the linesegment-scanline intersection.
 *
 *
 * Note that:
 * (1) Micromovements:
 *   In the current ZigZag implementation, there can be some micro movements for the zag connectors.
 *   The points on a zag connector are first collected and then added to the result polygon when this
 *   zag connector has for sure reached its end (i.e., all the points on this zag connector is known).
 *   Based on the model, a zag connector can contain a lot of tiny lines which can cause micro movements
 *   if we added them all.
 *
 *   To resolve this issue, we define a "minimum line length" for lines in a zag connector. If a line is
 *   shorter than the threshold, a second point (the "to" point) on this line will simply be ignored and
 *   the first point (the "from" point) will be kept as the starting point until there is a line that is
 *   long enough, and then that line will be added.
 */
class ZigzagConnectorProcessor 
{
public:
    /*!
     * Constructor.
     * 
     * \param rotation_matrix The rotation matrix used to enforce the infill angle
     * \param result The resulting line segments (Each line segment is a Polygon with 2 points)
     * \param use_endpieces Whether to include end pieces or not
     * \param connected_endpieces Whether the end pieces should be connected with the rest part of the infill
     * \param skip_some_zags Whether to skip some zags
     * \param zag_skip_count Skip 1 zag in every N zags
     */
    ZigzagConnectorProcessor(const PointMatrix& rotation_matrix, Polygons& result,
                             bool use_endpieces, bool connected_endpieces,
                             bool skip_some_zags, int zag_skip_count)
    : rotation_matrix(rotation_matrix)
    , result(result)
    , use_endpieces(use_endpieces)
    , connected_endpieces(connected_endpieces)
    , skip_some_zags(skip_some_zags)
    , zag_skip_count(zag_skip_count)
    , is_first_connector(true)
    , first_connector_end_scanline_index(0)
    , last_connector_index(0)
    {}

    virtual ~ZigzagConnectorProcessor()
    {}

    /*!
     * Handle the next vertex on the outer boundary.
     * \param vertex The vertex
     */
    virtual void registerVertex(const Point& vertex);

    /*!
     * Handle the next intersection between a scanline and the outer boundary.
     * 
     * \param intersection The intersection
     * \param scanline_index Index of the current scanline
     */
    virtual void registerScanlineSegmentIntersection(const Point& intersection, int scanline_index);

    /*!
     * Handle the end of a polygon and prepare for the next.
     * This function should reset all member variables.
     */
    virtual void registerPolyFinished();

protected:
    /*!
     * Reset the state so it can be used for processing another polygon.
     */
    void reset();

    /*!
     * Add a line to the result but not applying the rotation matrix.
     * 
     * \param from The one end of the line segment
     * \param to The other end of the line segment
     */
    void addLine(Point from, Point to);

    /*!
     * Checks whether the current connector should be added or not.
     *
     * \param start_scanline_idx the start scanline index of this scanline segment
     * \param end_scanline_idx The the end scanline index of this scanline segment
     */
    bool shouldAddCurrentConnector(int start_scanline_idx, int end_scanline_idx) const;

    /*!
     * Checks whether two points are separated at least by "threshold" microns.
     * If they are far away from each other enough, the line represented by the two points
     * will be added; In case they are close, the second point will be set to be the same
     * as the first and this line won't be added.
     *
     * \param first_point The first of the points
     * \param second_point The second of the points
     */
    void checkAndAddZagConnectorLine(Point* first_point, Point* second_point);

    /*!
     * Adds a Zag connector represented by the given points. The last line of the connector will not be
     * added if the given connector is an end piece and "connected_endpieces" is not enabled.
     *
     * \param points All the points on this connector
     * \param is_endpiece Whether this connector is an end piece
     */
    void addZagConnector(std::vector<Point>& points, bool is_endpiece);

protected:
    const PointMatrix& rotation_matrix; //!< The rotation matrix used to enforce the infill angle
    Polygons& result; //!< The result of the computation

    bool use_endpieces; //!< Whether to include end pieces or not
    bool connected_endpieces; //!< Whether the end pieces should be connected with the rest part of the infill
    int skip_some_zags; //!< Whether to skip some zags
    int zag_skip_count; //!< Skip 1 zag in every N zags

    bool is_first_connector; //!< indicating whether we are still looking for the first connector or not
    int first_connector_end_scanline_index; //!< scanline segment index of the first connector
    int last_connector_index; //!< scanline segment index of the last connector

    /*!
     * The line segments belonging the zigzag connector to which the very first vertex belongs.
     * This will be combined with the last handled zigzag_connector, which combine to a whole zigzag connector.
     *
     * Because the boundary polygon may start in in the middle of a zigzag connector,
     */
    std::vector<Point> first_connector;
    /*!
     * The currently built up zigzag connector (not the first/last) or end piece or discarded boundary segment
     */
    std::vector<Point> current_connector;
};

//
// Inline functions
//

inline void ZigzagConnectorProcessor::reset()
{
    is_first_connector = true;
    first_connector_end_scanline_index = 0;
    last_connector_index = 0;
    first_connector.clear();
    current_connector.clear();
}

inline void ZigzagConnectorProcessor::addLine(Point from, Point to)
{
    result.addLine(rotation_matrix.unapply(from), rotation_matrix.unapply(to));
}


} // namespace cura


#endif // INFILL_ZIGZAG_CONNECTOR_PROCESSOR_H
