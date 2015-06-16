#ifndef AABB_H
#define AABB_H

#include <limits>

#include "intpoint.h"

/*!
An Axis Aligned Bounding Box. Has a min and max vector, representing minimal and maximal coordinates in the three axes.
*/
struct AABB 
{
    Point3 min;
    Point3 max;
    
    AABB() 
    : min(std::numeric_limits<int32_t>::max(), std::numeric_limits<int32_t>::max(), std::numeric_limits<int32_t>::max())
    , max(std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min())
    {   
    }
    
    void include(Point3 p)
    {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
        max.z = std::max(max.z, p.z);   
    }
    
    void offset(Point3 offset)
    {
        min += offset;
        max += offset;
    }
    void offset(Point offset)
    {
        min += offset;
        max += offset;
    }
};


#endif//AABB_H

