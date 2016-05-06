/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef AABB_H
#define AABB_H


#include <limits>
#include "intpoint.h"
#include "polygon.h"


namespace cura
{
    
    
/* Axis aligned boundary box */
class AABB
{
public:
    Point min, max;

    AABB(); //!< initializes with invalid min and max
    AABB(Point& min, Point& max); //!< initializes with given min and max
    AABB(const Polygons& polys); //!< Computes the boundary box for the given polygons

    void calculate(const Polygons& polys); //!< Calculates the aabb for the given polygons (throws away old min and max data of this aabb)

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
    void include(Point point);

    /*!
     * Expand the borders of the bounding box in each direction with the given amount
     * 
     * \param dist The distance by which to expand the borders of the bounding box
     */
    void expand(int dist);
};

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
#endif//AABB_H

