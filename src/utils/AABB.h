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

    AABB()
    : min(POINT_MAX, POINT_MAX), max(POINT_MIN, POINT_MIN)
    {
    }
    AABB(Point&min, Point& max)
    : min(min), max(max)
    {
    }
    AABB(Polygons& polys)
    : min(POINT_MAX, POINT_MAX), max(POINT_MIN, POINT_MIN)
    {
        calculate(polys);
    }

    void calculate(Polygons& polys)
    {
        min = Point(POINT_MAX, POINT_MAX);
        max = Point(POINT_MIN, POINT_MIN);
        for(unsigned int i=0; i<polys.size(); i++)
        {
            for(unsigned int j=0; j<polys[i].size(); j++)
            {
                include(polys[i][j]);
            }
        }
    }

    bool hit(const AABB& other) const
    {
        if (max.X < other.min.X) return false;
        if (min.X > other.max.X) return false;
        if (max.Y < other.min.Y) return false;
        if (min.Y > other.max.Y) return false;
        return true;
    }
    
    /*!
     * \brief Includes the specified point in the bounding box.
     * 
     * The bounding box is expanded if the point is not within the bounding box.
     * 
     * \param point The point to include in the bounding box.
     */
    void include(Point point)
    {
        min.X = std::min(min.X,point.X);
        min.Y = std::min(min.Y,point.Y);
        max.X = std::max(max.X,point.X);
        max.Y = std::max(max.Y,point.Y);
    }
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
    AABB3D() 
    : min(std::numeric_limits<int32_t>::max(), std::numeric_limits<int32_t>::max(), std::numeric_limits<int32_t>::max())
    , max(std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min())
    {   
    }
    
    /*!
     * Expand the AABB3D to include the point \p p.
     * \param p The point to include with the bounding box.
     */
    void include(Point3 p)
    {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
        max.z = std::max(max.z, p.z);   
    }
    
    /*!
     * Offset the coordinates of the bounding box.
     * \param offset The offset with which to offset the AABB3D.
     */
    void offset(Point3 offset)
    {
        min += offset;
        max += offset;
    }
    /*!
     * Offset the coordinates of the bounding box.
     * \param offset The offset with which to offset the AABB3D.
     */
    void offset(Point offset)
    {
        min += offset;
        max += offset;
    }
};

}//namespace cura
#endif//AABB_H

