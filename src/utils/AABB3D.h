/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_AABB3D_H
#define UTILS_AABB3D_H


#include "intpoint.h"


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
     */
    void include(Point3 p);

    /*!
     * Offset the coordinates of the bounding box.
     * \param offset The offset with which to offset the AABB3D.
     */
    void offset(Point3 offset);

    /*!
     * Offset the coordinates of the bounding box.
     * \param offset The offset with which to offset the AABB3D.
     */
    void offset(Point offset);
};

}//namespace cura
#endif//UTILS_AABB3D_H

