/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_ZIGZAG_CONNECTOR_PROCESSOR_H
#define INFILL_ZIGZAG_CONNECTOR_PROCESSOR_H

#include "../utils/polygon.h"

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
 *
 * Each of these has a base class for which ZigzagConnectorProcessor is an ancestor.
 * The inheritance structure is as such:
 *                                                                      ZigzagConnectorProcessor
 *                                                                           /             \                                    .
 *                                                                          /               \                                   .
 *                                                  ActualZigzagConnectorProcessor      NoZigZagConnectorProcessor
 *                                                         /                \             for lines infill                                      .
 *                                                        /                  \                                                  .
 *                          ZigzagConnectorProcessorEndPieces   ZigzagConnectorProcessorNoEndPieces
 *                                     /            \                 for zigzag infill (without end pieces)                                                          .
 *                                    /              \                                                                          .
 * ZigzagConnectorProcessorConnectedEndPieces     ZigzagConnectorProcessorDisconnectedEndPieces
 * for zigzag support with normal endpieces          for zigzag support with disconnected endpieces for more easy removability
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
 */
class ZigzagConnectorProcessor
{
protected:
    const PointMatrix& rotation_matrix; //!< The rotation matrix used to enforce the infill angle
    Polygons& result; //!< The result of the computation

    virtual ~ZigzagConnectorProcessor()
    {}

    /*!
     * Add a line to the result bu unapplying the rotation rotation_matrix.
     *
     * \param from The one end of the line segment
     * \param to The other end of the line segment
     */
    void addLine(Point from, Point to)
    {
        result.addLine(rotation_matrix.unapply(from), rotation_matrix.unapply(to));
    }

    /*!
     * Basic constructor. Inheriting children should call this constructor.
     *
     * \param rotation_matrix The rotation matrix used to enforce the infill angle
     * \param result The resulting line segments (Each line segment is a Polygon with 2 points)
     */
    ZigzagConnectorProcessor(const PointMatrix& rotation_matrix, Polygons& result)
    : rotation_matrix(rotation_matrix)
    , result(result)
    {}
public:

    /*!
     * Handle the next vertex on the outer boundary.
     * \param vertex The vertex
     */
    virtual void registerVertex(const Point& vertex) = 0;

    /*!
     * Handle the next intersection between a scanline and the outer boundary.
     *
     * \param intersection The intersection
     * \param scanline_is_even Whether the scanline was even
     */
    virtual void registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even) = 0;

    /*!
     * Handle the end of a polygon and prepare for the next.
     * This function should reset all member variables.
     */
    virtual void registerPolyFinished() = 0;
};


} // namespace cura


#endif // INFILL_ZIGZAG_CONNECTOR_PROCESSOR_H