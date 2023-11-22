//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/PolylineStitcher.h"

#include "utils/ExtrusionLine.h"
#include "utils/polygon.h"

namespace cura
{

template<>
bool PolylineStitcher<VariableWidthLines, ExtrusionLine, ExtrusionJunction>::canReverse(const PathsPointIndex<VariableWidthLines>& ppi)
{
    if ((*ppi.polygons_)[ppi.poly_idx_].is_odd_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

template<>
bool PolylineStitcher<Polygons, Polygon, Point>::canReverse(const PathsPointIndex<Polygons>&)
{
    return true;
}

template<>
bool PolylineStitcher<VariableWidthLines, ExtrusionLine, ExtrusionJunction>::canConnect(const ExtrusionLine& a, const ExtrusionLine& b)
{
    return a.is_odd_ == b.is_odd_;
}

template<>
bool PolylineStitcher<Polygons, Polygon, Point>::canConnect(const Polygon&, const Polygon&)
{
    return true;
}

template<>
bool PolylineStitcher<VariableWidthLines, ExtrusionLine, ExtrusionJunction>::isOdd(const ExtrusionLine& line)
{
    return line.is_odd_;
}

template<>
bool PolylineStitcher<Polygons, Polygon, Point>::isOdd(const Polygon&)
{
    return false;
}

}//namespace cura

