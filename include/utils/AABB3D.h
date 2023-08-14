// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_AABB3D_H
#define UTILS_AABB3D_H

#include "IntPoint.h"
#include "utils/AABB.h"

namespace cura
{

/*!
An Axis Aligned Bounding Box. Has a min and max vector, representing minimal and maximal coordinates in the three axes.
*/
struct AABB3D
{
    Point3 min; //!< The minimal coordinates in x, y and z direction
    Point3 max; //!< The maximal coordinates in x, y and z direction

    /*!
     * Create an AABB3D with coordinates at the numeric limits.
     */
    AABB3D();

    /*!
     * Create an AABB3D with given limits
     */
    AABB3D(Point3 min, Point3 max);

    /*!
     * Get the middle of the bounding box
     */
    Point3 getMiddle() const;

    /*!
     * Creates a 2D version of this bounding box by leaving away the Z
     * component.
     *
     * \return A 2D version of this bounding box.
     */
    AABB flatten() const;

    /*!
     * Check whether this aabb overlaps with another.
     *
     * In the boundary case false is returned.
     *
     * \param other the aabb to check for overlaps with
     * \return Whether the two aabbs overlap
     */
    bool hit(const AABB3D& other) const;

    /*!
     * Expand the AABB3D to include the point \p p.
     * \param p The point to include with the bounding box.
     * \return this object (which has changed)
     */
    AABB3D include(Point3 p);

    /*!
     * Expand the AABB3D to include the bounding box \p aabb.
     * \param aabb The aabb to include with this bounding box.
     * \return this object (which has changed)
     */
    AABB3D include(const AABB3D& aabb);

    /*!
     * Expand the AABB3D to include a z-coordinate.
     *
     * This is for including a point of which the X and Y coordinates are
     * unknown but known to already be included in the bounding box.
     * \return this object (which has changed)
     */
    AABB3D includeZ(coord_t z);

    /*!
     * Offset the coordinates of the bounding box.
     * \param offset The offset with which to offset the AABB3D.
     * \return this object (which has changed)
     */
    AABB3D translate(Point3 offset);

    /*!
     * Offset the coordinates of the bounding box.
     * \param offset The offset with which to offset the AABB3D.
     * \return this object (which has changed)
     */
    AABB3D translate(Point offset);

    /*!
     * Offset the bounding box in the horizontal direction; outward or inward.
     *
     * \param outset the distance (positive or negative) to expand the bounding box outward
     * \return this object (which has changed)
     */
    AABB3D expand(coord_t outset);

    /*!
     * Offset the bounding box in the horizontal direction; outward or inward.
     *
     * \param outset the distance (positive or negative) to expand the bounding box outward
     * \return this object (which has changed)
     */
    AABB3D expandXY(coord_t outset);
};

} // namespace cura
#endif // UTILS_AABB3D_H
