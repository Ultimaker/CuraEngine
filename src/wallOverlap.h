/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef WALL_OVERLAP_H
#define WALL_OVERLAP_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <list>

#include <functional> // hash function object

#include "utils/intpoint.h"
#include "utils/polygon.h"
#include "utils/linearAlg2D.h"
#include "utils/SymmetricPair.h"

#include "utils/ProximityPointLink.h"
#include "utils/PolygonProximityLinker.h"

namespace cura 
{

/*!
 * Class for computing and compensating for overlapping (outer) wall lines.
 * The overlapping area is approximated with connected trapzoids.
 * All places where the wall is closer than the nozzle width to another piece of wall are recorded.
 * The area of a trapezoid is then the length between two such locations multiplied by the average overlap at the two locations.
 * 
 * The amount of overlap between two locations is recorded in a link, so that we can look up the overlap at a given point in the polygon.
 * A link always occurs between a point already on a polygon and either another point of a polygon or a point on a line segment of a polygon.
 * In the latter case we insert the point into the polygon so that we can later look up by how much to reduce the extrusion at the corresponding line segment.
 * 
 * At the end of a sequence of trapezoids the overlap area generally ends with a residual triangle.
 * Therefore points are introduced on the line segments involved and a link is created with overlap zero.
 * 
 * \see PolygonProximityLinker
 * 
 * Each point on the polygons then maps to a link, so that we can easily look up which links corresponds 
 * to the current line segment being produced when producing gcode.
 * 
 * When producing gcode, the first line crossing the overlap area is laid down normally and the second line is reduced by the overlap amount.
 * For this reason the function WallOverlapComputation::getFlow changes the internal state of the PolygonProximityLinker.
 * 
 * The main functionality of this class is performed by the constructor, by calling the constructor of PolygonProximityLinker.
 * The adjustment during gcode generation is made with the help of WallOverlapComputation::getFlow
 */
class WallOverlapComputation
{
    PolygonProximityLinker overlap_linker;
    int64_t line_width;

    std::unordered_set<SymmetricPair<ProximityPointLink>> passed_links;
public:
    /*!
     * Compute the flow for a given line segment in the wall.
     * 
     * \warning the first time this function is called it returns a different thing than the second, because the second time it thinks it already passed this segment once.
     * 
     * \param from The beginning of the line segment
     * \param to The ending of the line segment
     * \return a value between zero and one representing the reduced flow of the line segment
     */
    float getFlow(const Point& from, const Point& to);

    /*!
     * Computes the neccesary priliminaries in order to efficiently compute the flow when generatign gcode paths.
     * \param polygons The wall polygons for which to compute the overlaps
     */
    WallOverlapComputation(Polygons& polygons, int lineWidth);

private:
    /*!
     * Check whether \p from_it and \p from_other_it are connected and if so,
     * return the overlap area between those and the link \p to_link
     * 
     * This presupposes that \p to_link and the link from \p from_it to \p from_other_it forms a single overlap quadrilateral
     * 
     * from_other         to_other
     *          o<--------o
     *          ?         :
     *          ?         :
     *          ?         :
     *          o-------->o
     *       from         to
     * 
     * \param from_it The first point possibly invovled in the second link
     * \param to_it The first point of \p to_link connected to \p from_it
     * \param to_link The first link involved in the overlap: from \p from_it to \p to_it
     * \param from_other_it The second point possibly involved in the second link
     * \param to_other_it The second point of \p to_link connected to \p from_other_it
     * \return The overlap area between the two links, or zero if there was no such link
     */
    int64_t handlePotentialOverlap(const ListPolyIt from_it, const ListPolyIt to_it, const ProximityPointLink& to_link, const ListPolyIt from_other_it, const ListPolyIt to_other_it);

    /*!
     * Compute the approximate overlap area between two line segments
     * or between a line segment and a point when one of the line segments has the same start as end point.
     * 
     *   other_to         other_from
     *          o<--------o
     *          :         :
     *          :,,,,,,,,,:
     *          ://///////: \
     *          ://///////:  } overlap area
     *          ://///////: /
     *          :''''''''':
     *          :         :
     *          o-------->o
     *       from         to
     * 
     * \param from The starting point of the one line segment
     * \param to the end point of the one line segment
     * \param to_dist The distance between \p to and \p to_other
     * \param other_from The starting point of the other line segment (across the overlap of \p to)
     * \param other_to The end point of the other line segment (across the overlap of \p from)
     * \param from_dist The distance between \p from and \p from_other
     */
    int64_t getApproxOverlapArea(const Point from, const Point to, const int64_t to_dist, const Point other_from, const Point other_to, const int64_t from_dist);

    /*!
     * Check whether an overlap segment between two consecutive links is already passed
     * 
     * \note \p link_a and \p link_b are assumed to be consecutive
     * 
     * \param link_a the one link of the overlap area
     * \param link_b the other link of the overlap area
     * \return whether the link has already been passed once
     */
    bool getIsPassed(const ProximityPointLink& link_a, const ProximityPointLink& link_b);

    /*!
     * Mark an overlap area between two consecutive links as being passed once already.
     * 
     * \note \p link_a and \p link_b are assumed to be consecutive
     * 
     * \param link_a the one link of the overlap area
     * \param link_b the other link of the overlap area
     */
    void setIsPassed(const ProximityPointLink& link_a, const ProximityPointLink& link_b);
};


}//namespace cura



#endif//WALL_OVERLAP_H
