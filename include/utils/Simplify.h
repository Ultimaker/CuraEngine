//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_SIMPLIFY_H
#define UTILS_SIMPLIFY_H

#include "polygon.h"
#include "ExtrusionLine.h"
#include "linearAlg2D.h" //To calculate line deviations and intersecting lines.
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
     * Simplify a batch of polygons.
     * \param polygons The polygons to simplify.
     * \return The simplified polygons.
     */
    Polygons polygon(const Polygons& polygons) const;

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
    Polygons polyline(const Polygons& polylines) const;

    /*!
     * Simplify a polyline.
     *
     * The endpoints of the polyline cannot be altered.
     * \param polyline The polyline to simplify.
     * \return The simplified polyline.
     */
    Polygon polyline(const Polygon& polyline) const;

    /*!
     * Simplify a variable-line-width polyline.
     *
     * The endpoints of the polyline cannot be altered.
     * \param polyline The polyline to simplify.
     * \return The simplified polyline.
     */
    ExtrusionLine polyline(const ExtrusionLine& polyline) const;

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

    template<typename Polygonal>
    bool detectSmall(const Polygonal& polygon, const coord_t& min_size) const
    {
        if (polygon.size() < min_size) //For polygon, 2 or fewer vertices is degenerate. Delete it. For polyline, 1 vertex is degenerate.
        {
            return true;
        }
        if (polygon.size() == min_size)
        {
            const auto a = getPosition(polygon[0]);
            const auto b = getPosition(polygon[1]);
            const auto c = getPosition(polygon[polygon.size() - 1]);
            if (std::max(std::max(vSize2(b - a), vSize2(c - a)), vSize2(c - b)) < min_resolution * min_resolution)
            {
                // ... unless they are degenetate.
                return true;
            }
        }
        return false;
    }

    /*!
     * The main simplification algorithm starts here.
     * \tparam Polygonal A polygonal object, which is a list of vertices.
     * \param polygon The polygonal chain to simplify.
     * \param is_closed Whether this is a closed polygon or an open polyline.
     * \return A simplified polygonal chain.
     */
    template<typename Polygonal>
    Polygonal simplify(const Polygonal& polygon, const bool is_closed) const
    {
        const size_t min_size = is_closed ? 3 : 2;
        if (detectSmall(polygon, min_size))
        {
            return createEmpty(polygon);
        }
        if(polygon.size() == min_size) //For polygon, don't reduce below 3. For polyline, not below 2.
        {
            return polygon;
        }

        std::vector<bool> to_delete(polygon.size(), false);
        auto comparator = [](const std::pair<size_t, coord_t>& vertex_a, const std::pair<size_t, coord_t>& vertex_b)
        {
            return vertex_a.second > vertex_b.second || (vertex_a.second == vertex_b.second && vertex_a.first > vertex_b.first);
        };
        std::priority_queue<std::pair<size_t, coord_t>, std::vector<std::pair<size_t, coord_t>>, decltype(comparator)> by_importance(comparator);

        Polygonal result = polygon; //Make a copy so that we can also shift vertices.
        for (int64_t current_removed = -1; (polygon.size() - current_removed) > min_size && current_removed != 0;)
        {
            current_removed = 0;

            //Add the initial points.
            for (size_t i = 0; i < result.size(); ++i)
            {
                if (to_delete[i])
                {
                    continue;
                }
                const coord_t vertex_importance = importance(result, to_delete, i, is_closed);
                by_importance.emplace(i, vertex_importance);
            }

            //Iteratively remove the least important point until a threshold.
            coord_t vertex_importance = 0;
            while (! by_importance.empty() && (polygon.size() - current_removed) > min_size)
            {
                std::pair<size_t, coord_t> vertex = by_importance.top();
                by_importance.pop();
                //The importance may have changed since this vertex was inserted. Re-compute it now.
                //If it doesn't change, it's safe to process.
                vertex_importance = importance(result, to_delete, vertex.first, is_closed);
                if (vertex_importance != vertex.second)
                {
                    by_importance.emplace(vertex.first, vertex_importance); //Re-insert with updated importance.
                    continue;
                }

                if (vertex_importance <= max_deviation * max_deviation)
                {
                    current_removed += remove(result, to_delete, vertex.first, vertex_importance, is_closed) ? 1 : 0;
                }
            }
        }

        //Now remove the marked vertices in one sweep.
        Polygonal filtered = createEmpty(polygon);
        for(size_t i = 0; i < result.size(); ++i)
        {
            if(!to_delete[i])
            {
                appendVertex(filtered, result[i]);
            }
        }

        if (detectSmall(filtered, min_size))
        {
            return createEmpty(filtered);
        }
        return filtered;
    }

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
    coord_t importance(const Polygonal& polygon, const std::vector<bool>& to_delete, const size_t index, const bool is_closed) const
    {
        const size_t poly_size = polygon.size();
        if(!is_closed && (index == 0 || index == poly_size - 1))
        {
            return std::numeric_limits<coord_t>::max(); //Endpoints of the polyline must always be retained.
        }
        //From here on out we can safely look at the vertex neighbors and assume it's a polygon. We won't go out of bounds of the polyline.

        const Point& vertex = getPosition(polygon[index]);
        const size_t before_index = previousNotDeleted(index, to_delete);
        const size_t after_index = nextNotDeleted(index, to_delete);

        const coord_t area_deviation = getAreaDeviation(polygon[before_index], polygon[index], polygon[after_index]);
        if(area_deviation > max_area_deviation) //Removing this line causes the variable line width to get flattened out too much.
        {
            return std::numeric_limits<coord_t>::max();
        }

        const Point& before = getPosition(polygon[before_index]);
        const Point& after = getPosition(polygon[after_index]);
        const coord_t deviation2 = LinearAlg2D::getDist2FromLine(vertex, before, after);
        if(deviation2 <= min_resolution * min_resolution) //Deviation so small that it's always desired to remove them.
        {
            return deviation2;
        }
        if(vSize2(before - vertex) > max_resolution * max_resolution && vSize2(after - vertex) > max_resolution * max_resolution)
        {
            return std::numeric_limits<coord_t>::max(); //Long line segments, no need to remove this one.
        }
        return deviation2;
    }

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
    bool remove(Polygonal& polygon, std::vector<bool>& to_delete, const size_t vertex, const coord_t deviation2, const bool is_closed) const
    {
        if(deviation2 <= min_resolution * min_resolution)
        {
            //At less than the minimum resolution we're always allowed to delete the vertex.
            //Even if the adjacent line segments are very long.
            to_delete[vertex] = true;
            return true;
        }

        const size_t before = previousNotDeleted(vertex, to_delete);
        const size_t after = nextNotDeleted(vertex, to_delete);
        const Point& vertex_position = getPosition(polygon[vertex]);
        const Point& before_position = getPosition(polygon[before]);
        const Point& after_position = getPosition(polygon[after]);
        const coord_t length2_before = vSize2(vertex_position - before_position);
        const coord_t length2_after = vSize2(vertex_position - after_position);

        if(length2_before <= max_resolution * max_resolution && length2_after <= max_resolution * max_resolution) //Both adjacent line segments are short.
        {
            //Removing this vertex does little harm. No long lines will be shifted.
            to_delete[vertex] = true;
            return true;
        }

        //Otherwise, one edge next to this vertex is longer than max_resolution. The other is shorter.
        //In this case we want to remove the short edge by replacing it with a vertex where the two surrounding edges intersect.
        //Find the two line segments surrounding the short edge here ("before" and "after" edges).
        Point before_from, before_to, after_from, after_to;
        if(length2_before <= length2_after) //Before is the shorter line.
        {
            if(!is_closed && before == 0) //No edge before the short edge.
            {
                return false; //Edge cannot be deleted without shifting a long edge. Don't remove anything.
            }
            const size_t before_before = previousNotDeleted(before, to_delete);
            before_from = getPosition(polygon[before_before]);
            before_to = getPosition(polygon[before]);
            after_from = getPosition(polygon[vertex]);
            after_to = getPosition(polygon[after]);
        }
        else
        {
            if(!is_closed && after == polygon.size() - 1) //No edge after the short edge.
            {
                return false; //Edge cannot be deleted without shifting a long edge. Don't remove anything.
            }
            const size_t after_after = nextNotDeleted(after, to_delete);
            before_from = getPosition(polygon[before]);
            before_to = getPosition(polygon[vertex]);
            after_from = getPosition(polygon[after]);
            after_to = getPosition(polygon[after_after]);
        }
        Point intersection;
        const bool did_intersect = LinearAlg2D::lineLineIntersection(before_from, before_to, after_from, after_to, intersection);
        if(!did_intersect) //Lines are parallel.
        {
            return false; //Cannot remove edge without shifting a long edge. Don't remove anything.
        }
        const coord_t intersection_deviation = LinearAlg2D::getDist2FromLineSegment(before_to, intersection, after_from);
        if(intersection_deviation <= max_deviation * max_deviation) //Intersection point doesn't deviate too much. Use it!
        {
            to_delete[vertex] = true;
            polygon[length2_before <= length2_after ? before : after] = createIntersection(polygon[before], intersection, polygon[after]);
            return true;
        }
        return false;
    }

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
     * Create an empty polygon with the same properties as an original polygon,
     * but without the vertex data.
     * \param original The polygon to copy the properties from.
     * \return An empty polygon.
     */
    Polygon createEmpty(const Polygon& original) const;

    /*!
     * Create an empty extrusion line with the same properties as an original
     * extrusion line, but without the vertex data.
     * \param original The extrusion line to copy the properties from.
     * \return An empty extrusion line.
     */
    ExtrusionLine createEmpty(const ExtrusionLine& original) const;

    /*!
     * Append a vertex to this polygon.
     *
     * This function overloads to allow adding to all supported polygonal types.
     * \param polygon The polygon to add to.
     * \param vertex The vertex to add.
     */
    void appendVertex(Polygon& polygon, const Point& vertex) const;

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
    const Point& getPosition(const Point& vertex) const;

    /*!
     * Get the coordinates of a vertex.
     *
     * This overload is for the vertices of an ExtrusionJunction.
     * \param vertex A vertex to get the coordinates of.
     * \return The coordinates of that vertex.
     */
    const Point& getPosition(const ExtrusionJunction& vertex) const;

    /*!
     * Create an intersection vertex that can be placed in a polygon.
     * \param before One of the vertices of a removed edge. Unused in this
     * overload.
     * \param intersection The position of the intersection.
     * \param after One of the vertices of a removed edge. Unused in this
     * overload.
     */
    Point createIntersection(const Point& before, const Point intersection, const Point& after) const;

    /*!
     * Create an intersection vertex that can be placed in an ExtrusionLine.
     * \param before One of the vertices of the edge that gets replaced by an
     * intersection vertex.
     * \param intersection The position of the new intersection vertex.
     * \param after One of the vertices of the edge that gets replaced by an
     * intersection vertex.
     */
    ExtrusionJunction createIntersection(const ExtrusionJunction& before, const Point intersection, const ExtrusionJunction& after) const;

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
    coord_t getAreaDeviation(const Point& before, const Point& vertex, const Point& after) const;

    /*!
     * Get the extrusion area deviation that would be caused by removing this
     * vertex.
     * \param before The vertex before the one that is to be removed.
     * \param vertex The vertex that is to be removed.
     * \param after The vertex after the one that is to be removed.
     * \return The area deviation that would be caused by removing the vertex.
     */
    coord_t getAreaDeviation(const ExtrusionJunction& before, const ExtrusionJunction& vertex, const ExtrusionJunction& after) const;
};

} //namespace cura

#endif //UTILS_SIMPLIFY_H
