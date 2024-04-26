// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_SIMPLIFY_H
#define UTILS_SIMPLIFY_H

#include "geometry/Point2LL.h"
#include "utils/Coord_t.h"


namespace cura
{

template<class LineType>
class LinesSet;
struct ExtrusionLine;
struct ExtrusionJunction;
class MixedLinesSet;
class Settings;
class Shape;
class Polygon;
class OpenPolyline;
class ClosedPolyline;
class Polyline;

/*!
 * Utility class to reduce the resolution of polygons and polylines, under
 * certain constraints.
 *
 * This class implements a polygonal decimation algorithm which is meant to
 * reduce the resolution of polylines or polygons. This class provides several
 * methods to simplify different geometrical objects such that they can be
 * printed without buffer underruns in a 3D printer. The simplified results have
 * the following constraints:
 * * The simplified path does not deviate more than the Maximum Deviation from
 *   the original path.
 * * In variable-width lines, the simplified path may not deviate more than the
 *   Maximum Area Deviation from the original path in the area that each line
 *   segment covers (width * length). This does not mean that the line couldn't
 *   be moved, only that its width may not locally be adjusted too much.
 * * The simplified path does not contain line segments shorter than the Maximum
 *   Resolution, unless that interferes with the first two criteria.
 * * The simplified path does not contain any vertices where removing it would
 *   cause a deviation of less than 5 micron.
 * * The simplified path does not contain any line segments shorter than 5
 *   micron.
 * * Line segments significantly longer than the Maximum Resolution do not get
 *   moved for the bigger part of their length by more than 5 micron, not even
 *   if this would be allowable by the Maximum Deviation.
 */
class Simplify
{
public:
    /*!
     * Line segments shorter than this size should be considered for removal.
     */
    coord_t max_resolution_;

    /*!
     * If removing a vertex causes a deviation further than this, it may not be
     * removed.
     */
    coord_t max_deviation_;

    /*!
     * If removing a vertex causes the covered area of the line segments to
     * change by more than this, it may not be removed.
     */
    coord_t max_area_deviation_;

    /*!
     * Construct a simplifier, storing the simplification parameters in the
     * instance (as a factory pattern).
     * \param max_resolution Line segments smaller than this are considered for
     * joining with other line segments.
     * \param max_deviation If removing a vertex would cause a deviation larger
     * than this, it cannot be removed.
     * \param max_area_deviation If removing a vertex would cause the covered
     * area in total to change more than this, it cannot be removed.
     */
    Simplify(const coord_t max_resolution, const coord_t max_deviation, const coord_t max_area_deviation);

    /*!
     * Construct a simplifier using the resolution settings inside of the given
     * settings object.
     * \param settings A settings object to obtain the simplification parameters
     * from.
     */
    Simplify(const Settings& settings);

    /*!
     * Simplify a batch of polygons.
     * \param polygons The polygons to simplify.
     * \return The simplified polygons.
     */
    Shape polygon(const Shape& polygons) const;

    /*!
     * Simplify a polygon.
     * \param polygon The polygon to simplify.
     * \return The simplified polygon.
     */
    Polygon polygon(const Polygon& polygon) const;

    /*!
     * Simplify a variable-line-width polygon.
     * \param polygon The polygon to simplify.
     * \return The simplified polygon.
     */
    ExtrusionLine polygon(const ExtrusionLine& polygon) const;

    /*!
     * Simplify a batch of polylines.
     *
     * The endpoints of each polyline cannot be altered.
     * \param polylines The polylines to simplify.
     * \return The simplified polylines.
     */
    template<class LineType>
    LinesSet<LineType> polyline(const LinesSet<LineType>& polylines) const;

    /*!
     * Simplify a batch of polylines.
     *
     * The endpoints of each polyline cannot be altered.
     * \param polylines The polylines to simplify.
     * \return The simplified polylines.
     */
    MixedLinesSet polyline(const MixedLinesSet& polylines) const;

    /*!
     * Simplify a polyline.
     *
     * The endpoints of the polyline cannot be altered.
     * \param polyline The polyline to simplify.
     * \return The simplified polyline.
     */
    OpenPolyline polyline(const OpenPolyline& polyline) const;

    /*!
     * Simplify a polyline.
     *
     * \param polyline The polyline to simplify.
     * \return The simplified polyline.
     */
    ClosedPolyline polyline(const ClosedPolyline& polyline) const;

    /*!
     * Simplify a variable-line-width polyline.
     *
     * The endpoints of the polyline cannot be altered.
     * \param polyline The polyline to simplify.
     * \return The simplified polyline.
     */
    ExtrusionLine polyline(const ExtrusionLine& polyline) const;

protected:
    /*!
     * Line segments smaller than this should not occur in the output.
     * If a vertex causes deviation of less than this, it should always be
     * removed.
     */
    constexpr static coord_t min_resolution = 5; // 5 units, regardless of how big those are, to allow for rounding errors.

    /*!
     * Helper method to find the index of the next vertex that is not about to
     * get deleted.
     *
     * This method assumes that the polygon is looping. If it is a polyline, the
     * endpoints of the polyline may never be deleted so it should never be an
     * issue.
     * \param index The index of the current vertex.
     * \param to_delete For each vertex, whether it is to be deleted.
     * \return The index of the vertex afterwards.
     */
    size_t nextNotDeleted(size_t index, const std::vector<bool>& to_delete) const;

    /*!
     * Helper method to find the index of the previous vertex that is not about
     * to get deleted.
     *
     * This method assumes that the polygon is looping. If it is a polyline, the
     * endpoints of the polyline may never be deleted so it should never be an
     * issue.
     * \param index The index of the current vertex.
     * \param to_delete For each vertex, whether it is to be deleted.
     * \return The index of the vertex before it.
     */
    size_t previousNotDeleted(size_t index, const std::vector<bool>& to_delete) const;

    /*!
     * Append a vertex to this polygon.
     *
     * This function overloads to allow adding to all supported polygonal types.
     * \param polygon The polygon to add to.
     * \param vertex The vertex to add.
     */
    void appendVertex(Polyline& polygon, const Point2LL& vertex) const;

    /*!
     * Append a vertex to this extrusion line.
     *
     * This function overloads to allow adding to all supported polygonal types.
     * \param extrusion_line The extrusion line to add to.
     * \param vertex The vertex to add.
     */
    void appendVertex(ExtrusionLine& extrusion_line, const ExtrusionJunction& vertex) const;

    /*!
     * Get the coordinates of a vertex.
     *
     * This overload is for the vertices of a Polygon, which are simply the
     * coordinates themselves. So this function is the identity.
     * \param vertex A vertex to get the coordinates of.
     * \return The coordinates of that vertex.
     */
    const Point2LL& getPosition(const Point2LL& vertex) const;

    /*!
     * Get the coordinates of a vertex.
     *
     * This overload is for the vertices of an ExtrusionJunction.
     * \param vertex A vertex to get the coordinates of.
     * \return The coordinates of that vertex.
     */
    const Point2LL& getPosition(const ExtrusionJunction& vertex) const;

    /*!
     * Create an intersection vertex that can be placed in a polygon.
     * \param before One of the vertices of a removed edge. Unused in this
     * overload.
     * \param intersection The position of the intersection.
     * \param after One of the vertices of a removed edge. Unused in this
     * overload.
     */
    Point2LL createIntersection(const Point2LL& before, const Point2LL intersection, const Point2LL& after) const;

    /*!
     * Create an intersection vertex that can be placed in an ExtrusionLine.
     * \param before One of the vertices of the edge that gets replaced by an
     * intersection vertex.
     * \param intersection The position of the new intersection vertex.
     * \param after One of the vertices of the edge that gets replaced by an
     * intersection vertex.
     */
    ExtrusionJunction createIntersection(const ExtrusionJunction& before, const Point2LL intersection, const ExtrusionJunction& after) const;

    /*!
     * Get the extrusion area deviation that would be caused by removing this
     * vertex.
     *
     * This is the overload for fixed-width polygons. Because those don't have
     * variable line width, the deviation is always 0.
     * \param before The vertex before the one that is to be removed.
     * \param vertex The vertex that is to be removed.
     * \param after The vertex after the one that is to be removed.
     * \return The area deviation that would be caused by removing the vertex.
     */
    coord_t getAreaDeviation(const Point2LL& before, const Point2LL& vertex, const Point2LL& after) const;

    /*!
     * Get the extrusion area deviation that would be caused by removing this
     * vertex.
     * \param before The vertex before the one that is to be removed.
     * \param vertex The vertex that is to be removed.
     * \param after The vertex after the one that is to be removed.
     * \return The area deviation that would be caused by removing the vertex.
     */
    coord_t getAreaDeviation(const ExtrusionJunction& before, const ExtrusionJunction& vertex, const ExtrusionJunction& after) const;

private:
    /*!
     * Create an empty polygonal with the same properties as an original polygon,
     * but without the vertex data.
     * \param original The polygonal to copy the properties from.
     * \return An empty polygonal.
     */
    template<typename Polygonal>
    static Polygonal createEmpty(const Polygonal& original);

    template<typename Polygonal>
    bool detectSmall(const Polygonal& polygon, const coord_t& min_size) const;

    /*!
     * The main simplification algorithm starts here.
     * \tparam Polygonal A polygonal object, which is a list of vertices.
     * \param polygon The polygonal chain to simplify.
     * \param is_closed Whether this is a closed polygon or an open polyline.
     * \return A simplified polygonal chain.
     */
    template<typename Polygonal>
    Polygonal simplify(const Polygonal& polygon, const bool is_closed) const;

    /*!
     * A measure of the importance of a vertex.
     * \tparam Polygonal A polygonal object, which is a list of vertices.
     * \param polygon The polygon or polyline the vertex is part of.
     * \param to_delete For each vertex, whether it is set to be deleted.
     * \param index The vertex index to compute the importance of.
     * \param is_closed Whether the polygon is closed (a polygon) or open
     * (a polyline).
     * \return A measure of how important the vertex is. Higher importance means
     * that the vertex should probably be retained in the output.
     */
    template<typename Polygonal>
    coord_t importance(const Polygonal& polygon, const std::vector<bool>& to_delete, const size_t index, const bool is_closed) const;

    /*!
     * Mark a vertex for removal.
     *
     * This function looks in the vertex and the four edges surrounding it to
     * determine the best way to remove the given vertex. It may choose instead
     * to delete an edge, fusing two vertices together.
     * \tparam Polygonal A polygonal object, which is a list of vertices.
     * \param polygon The polygon to remove a vertex from.
     * \param to_delete The vertices that have been marked for deletion so far.
     * This will be edited in-place.
     * \param vertex The index of the vertex to remove.
     * \param deviation2 The previously found deviation for this vertex.
     * \param is_closed Whether we're working on a closed polygon or an open
     \return Whether something is actually removed
     * polyline.
     */
    template<typename Polygonal>
    bool remove(Polygonal& polygon, std::vector<bool>& to_delete, const size_t vertex, const coord_t deviation2, const bool is_closed) const;
};

} // namespace cura

#endif // UTILS_SIMPLIFY_H
