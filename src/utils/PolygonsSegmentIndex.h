//Copyright (c) 2018 Ultimaker B.V.


#ifndef UTILS_POLYGONS_SEGMENT_INDEX_H
#define UTILS_POLYGONS_SEGMENT_INDEX_H

#include <vector>

#include "PolygonsPointIndex.h"

namespace arachne 
{

/*!
 * A class for iterating over the points in one of the polygons in a \ref Polygons object
 */
class PolygonsSegmentIndex : public PolygonsPointIndex
{
public:
    PolygonsSegmentIndex()
    : PolygonsPointIndex()
    {
    }
    PolygonsSegmentIndex(const Polygons* polygons, unsigned int poly_idx, unsigned int point_idx)
    : PolygonsPointIndex(polygons, poly_idx, point_idx)
    {}

    Point from() const
    {
        return PolygonsPointIndex::p();
    }
    
    Point to() const
    {
        return PolygonsSegmentIndex::next().p();
    }
};


}//namespace arachne


#endif//UTILS_POLYGONS_SEGMENT_INDEX_H
