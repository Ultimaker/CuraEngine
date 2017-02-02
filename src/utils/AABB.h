/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_AABB_H
#define UTILS_AABB_H


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
    AABB(ConstPolygonRef poly); //!< Computes the boundary box for the given polygons

    void calculate(const Polygons& polys); //!< Calculates the aabb for the given polygons (throws away old min and max data of this aabb)
    void calculate(ConstPolygonRef poly); //!< Calculates the aabb for the given polygon (throws away old min and max data of this aabb)

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

}//namespace cura
#endif//UTILS_AABB_H

