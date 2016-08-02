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

#include "utils/ProximityPointLink.h"
#include "utils/PolygonProximityLinker.h"

#include "debug.h" // TODO remove

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

    std::unordered_set<ProximityPointLink> passed_links;
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
    float getFlow(Point& from, Point& to);

    /*!
     * Computes the neccesary priliminaries in order to efficiently compute the flow when generatign gcode paths.
     * \param polygons The wall polygons for which to compute the overlaps
     */
    WallOverlapComputation(Polygons& polygons, int lineWidth);

private:
    /*!
     * Check whether \p from_it and \p to_it are connected and if so,
     * return the overlap area between those and the link \p link_a
     * 
     * This presupposes that \p link_a and the link from \p from_it to \p to_it forms a single overlap quadrilateral
     * 
     * \param link_a The first link involved in the overlap
     * \param from_it The first point possibly invovled in the second link
     * \param to_it The second point possibly invovled in the second link
     * \return The overlap area between the two links, or zero if there was no such link
     */
    int64_t handlePotentialOverlap(const ProximityPointLink& link_a, const ListPolyIt from_it, const ListPolyIt to_it);
    /*!
     * Compute the approximate overlap area between two line segments
     */
    int64_t getApproxOverlapArea(const ProximityPointLink& from, const ProximityPointLink& to);
    /*!
     * Compute the approximate overlap area between two line segments
     */
    int64_t getApproxOverlapArea(const Point from_a, const Point from_b, const int64_t from_dist, const Point to_a, const Point to_b, const int64_t to_dist);

    /*!
     * \param link the link to check for
     * \return whether the link has already been passed once
     */
    bool getIsPassed(const ProximityPointLink& link);

    /*!
     * Mark the link as being passed once already
     * \param link the link to mark as passed
     */
    void setIsPassed(const ProximityPointLink& link);
};


}//namespace cura



#endif//WALL_OVERLAP_H
