//Copyright (c) 2019 Ultimaker B.V.


#ifndef TEST_GEOMETRY_PRESCRIBED_H
#define TEST_GEOMETRY_PRESCRIBED_H

#include <utility>

#include "../utils/IntPoint.h"
#include "../utils/logoutput.h"
#include "../utils/polygonUtils.h"
#include "../utils/ExtrusionSegment.h"
#include "../utils/polygon.h"

namespace arachne
{

class Prescribed
{
public:
    static Polygons fromDistances(const std::vector<Point>& height_data)
    {
        coord_t y = 0;
        Polygons ret;
        
        if (height_data.size() < 1) return ret;
        
        ExtrusionJunction prev(Point(height_data.front().X, y), height_data.front().Y, 0);
        for (size_t datum_idx = 1; datum_idx < height_data.size(); datum_idx++)
        {
            Point datum = height_data[datum_idx];
            ExtrusionJunction junction(Point(datum.X, y), datum.Y, 0);
            ExtrusionSegment segment(prev, junction, true, false);
            ret = ret.unionPolygons(segment.toPolygons());
            prev = junction;
        }
        return ret;
    };
};

} // namespace arachne
#endif // TEST_GEOMETRY_PRESCRIBED_H
