// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <initializer_list>

#include "geometry/Point2LL.h"
#include "types/AngleRadians.h"

namespace curaengine::geometry
{
class OpenLinesSet;
class PointsSet;
class Polygon;
class Shape;
} // namespace curaengine::geometry

namespace curaengine::utils
{


/* Axis aligned boundary box */
class AABB
{
public:
    geometry::Point2LL min_, max_;

    AABB(); //!< initializes with invalid min and max
    AABB(const geometry::Point2LL& min, const geometry::Point2LL& max); //!< initializes with given min and max
    AABB(const std::initializer_list<geometry::Point2LL>& points); //!< initializes with given points
    AABB(const geometry::Shape& shape); //!< Computes the boundary box for the given shape
    AABB(const geometry::OpenLinesSet& lines); //!< Computes the boundary box for the given lines
    AABB(const geometry::PointsSet& poly); //!< Computes the boundary box for the given polygons

    void calculate(const geometry::Shape& shape); //!< Calculates the aabb for the given shape (throws away old min and max data of this aabb)
    void calculate(const geometry::OpenLinesSet& lines); //!< Calculates the aabb for the given lines (throws away old min and max data of this aabb)
    void calculate(const geometry::PointsSet& poly); //!< Calculates the aabb for the given polygon (throws away old min and max data of this aabb)

    /*!
     * Whether the bounding box contains the specified point.
     * \param point The point to check whether it is inside the bounding box.
     * \return ``true`` if the bounding box contains the specified point, or
     * ``false`` otherwise.
     */
    bool contains(const geometry::Point2LL& point) const;

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
    geometry::Point2LL getMiddle() const;

    /*!
     * If point outside of bounding box: positive distance-squared to the bounding box edges, otherwise negative.
     */
    coord_t distanceSquared(const geometry::Point2LL& p) const;

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
    void include(const geometry::Point2LL& point);

    void include(const geometry::PointsSet& polygon);

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
    geometry::Polygon toPolygon() const;

    [[nodiscard]] coord_t width() const;

    [[nodiscard]] coord_t height() const;


    /**
     * Calculates the Minimum Area Oriented Bounding Box for a given Shape (polygon).
     * This follows the "Rotating Calipers" logic: the minimum area box must have
     * one side collinear with an edge of the convex hull.
     * * @param shape The input points (assumed to be the convex hull for optimal performance).
     * @return A tuple containing the local AABB (extents) and the rotation angle.
     */
    static std::tuple<AABB, types::AngleRadians> minimumAreaOrientedBoundingBox(const geometry::Shape& shape);
};

} // namespace curaengine::utils
