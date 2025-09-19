// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_AABB_H
#define UTILS_AABB_H

#include "geometry/OpenLinesSet.h"
#include "geometry/Point2LL.h"

namespace cura
{

class PointsSet;
class Polygon;
class Shape;

/* Axis aligned boundary box */
class AABB
{
public:
    Point2LL min_, max_;

    AABB(); //!< initializes with invalid min and max
    AABB(const Point2LL& min, const Point2LL& max); //!< initializes with given min and max
    AABB(const Shape& shape); //!< Computes the boundary box for the given shape
    AABB(const OpenLinesSet& lines); //!< Computes the boundary box for the given lines
    AABB(const PointsSet& poly); //!< Computes the boundary box for the given polygons

    void calculate(const Shape& shape); //!< Calculates the aabb for the given shape (throws away old min and max data of this aabb)
    void calculate(const OpenLinesSet& lines); //!< Calculates the aabb for the given lines (throws away old min and max data of this aabb)
    void calculate(const PointsSet& poly); //!< Calculates the aabb for the given polygon (throws away old min and max data of this aabb)

    /*!
     * Whether the bounding box contains the specified point.
     * \param point The point to check whether it is inside the bounding box.
     * \return ``true`` if the bounding box contains the specified point, or
     * ``false`` otherwise.
     */
    bool contains(const Point2LL& point) const;

    /*!
     * Whether this bounding box contains the other bounding box.
     */
    bool contains(const AABB& other) const;

    /*!
     * Returns the area of this bounding box.
     * Note: Area is negative for uninitialized, and 0 for empty.
     */
    coord_t area() const;

    /*!
     * Get the middle of the bounding box.
     */
    Point2LL getMiddle() const;

    /*!
     * If point outside of bounding box: positive distance-squared to the bounding box edges, otherwise negative.
     */
    coord_t distanceSquared(const Point2LL& p) const;

    /*!
     * If other aabb outside of this bounding box: positive distance-squared to the bounding box edges,
     * otherwise negative distance squared for the most inner point included.
     */
    coord_t distanceSquared(const AABB& other) const;

    /*!
     * Check whether this aabb overlaps with another.
     *
     * In the boundary case false is returned.
     *
     * \param other the aabb to check for overlaps with
     * \return Whether the two aabbs overlap
     */
    bool hit(const AABB& other) const;

    /*!
     * \brief Includes the specified point in the bounding box.
     *
     * The bounding box is expanded if the point is not within the bounding box.
     *
     * \param point The point to include in the bounding box.
     */
    void include(const Point2LL& point);

    void include(const PointsSet& polygon);

    /*!
     * \brief Includes the specified bounding box in the bounding box.
     *
     * The bounding box is expanded to include the other bounding box.
     *
     * This performs a union on two bounding boxes.
     *
     * \param other The bounding box to include in this one.
     */
    void include(const AABB& other);

    /*!
     * Expand the borders of the bounding box in each direction with the given amount
     *
     * \param dist The distance by which to expand the borders of the bounding box
     */
    void expand(int dist);

    /*!
     * Generate a square polygon which coincides with this aabb
     * \return the polygon of this aabb
     */
    Polygon toPolygon() const;
};

} // namespace cura
#endif // UTILS_AABB_H
