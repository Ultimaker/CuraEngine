//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_SIMPLIFY_H
#define UTILS_SIMPLIFY_H

#include "polygon.h"
#include "ExtrusionLine.h"
#include "../settings/Settings.h" //To load the parameters from a Settings object.

namespace cura
{

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
     * Simplify a polygon.
     * \param polygon The polygon to simplify.
     */
    Polygon polygon(const PolygonRef polygon);

    /*!
     * Simplify a variable-line-width polygon.
     * \param polygon The polygon to simplify.
     */
    ExtrusionLine polygon(const ExtrusionLine& polygon);

    /*!
     * Simplify a polyline.
     *
     * The endpoints of the polyline cannot be altered.
     * \param polyline The polyline to simplify.
     */
    Polygon polyline(const PolygonRef polyline);

    /*!
     * Simplify a variable-line-width polyline.
     * The endpoints of the polyline cannot be altered.
     * \param polyline The polyline to simplify.
     */
    ExtrusionLine polyline(const ExtrusionLine& polyline);

    /*!
     * Line segments shorter than this size should be considered for removal.
     */
    coord_t max_resolution;

    /*!
     * If removing a vertex causes a deviation further than this, it may not be
     * removed.
     */
    coord_t max_deviation;

    /*!
     * If removing a vertex causes the covered area of the line segments to
     * change by more than this, it may not be removed.
     */
    coord_t max_area_deviation;

protected:
    /*!
     * Line segments smaller than this should not occur in the output.
     * If a vertex causes deviation of less than this, it should always be
     * removed.
     */
    constexpr static coord_t min_resolution = 5; //5 units, regardless of how big those are, to allow for rounding errors.

    /*!
     * The main simplification algorithm starts here.
     * \param polygon The polygonal chain to simplify.
     * \param is_closed Whether this is a closed polygon or an open polyline.
     * \return A simplified polygonal chain.
     */
    Polygon simplify(const PolygonRef polygon, const bool is_closed);

    /*!
     * A measure of the importance of a vertex.
     * \param polygon The polygon or polyline the vertex is part of.
     * \param to_delete For each vertex, whether it is set to be deleted.
     * \param index The vertex index to compute the importance of.
     * \param is_closed Whether the polygon is closed (a polygon) or open
     * (a polyline).
     * \return A measure of how important the vertex is. Higher importance means
     * that the vertex should probably be retained in the output.
     */
    coord_t importance(const PolygonRef& polygon, const std::vector<bool>& to_delete, const size_t index, const bool is_closed) const;

    /*!
     * Mark a vertex for removal.
     *
     * This function looks in the vertex and the four edges surrounding it to
     * determine the best way to remove the given vertex. It may choose instead
     * to delete an edge, fusing two vertices together.
     * \param polygon The polygon to remove a vertex from.
     * \param to_delete The vertices that have been marked for deletion so far.
     * This will be edited in-place.
     * \param vertex The index of the vertex to remove.
     * \param deviation2 The previously found deviation for this vertex.
     * \param is_closed Whether we're working on a closed polygon or an open
     * polyline.
     */
    void remove(Polygon& polygon, std::vector<bool>& to_delete, const size_t vertex, const coord_t deviation2, const bool is_closed) const;

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
};

} //namespace cura

#endif //UTILS_SIMPLIFY_H
