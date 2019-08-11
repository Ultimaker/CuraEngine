//Copyright (c) 2019 Ultimaker B.V.


#ifndef TEST_GEOMETRY_VARIABLE_WIDTH_GCODE_TESTER_H
#define TEST_GEOMETRY_VARIABLE_WIDTH_GCODE_TESTER_H

#include <utility>

#include "../utils/ExtrusionSegment.h"

namespace arachne
{

class VariableWidthGcodeTester
{
public:
    static std::vector<std::list<ExtrusionLine>> zigzag(coord_t gap = MM2INT(2.0), coord_t size = MM2INT(50))
    {
        std::vector<std::list<ExtrusionLine>> result_polylines_per_index;
        result_polylines_per_index.resize(1);
        result_polylines_per_index[0].emplace_back(0, true);
        std::list<ExtrusionJunction>& polyline = result_polylines_per_index[0].front().junctions;
        
        coord_t normal_width = 400;
//         std::vector<float> widths({0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4});
        std::vector<float> widths({0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9});
        
        polyline.emplace_back(Point(-gap, size + normal_width / 2), normal_width, 0);
        polyline.emplace_back(Point(widths.size() * gap, size + normal_width / 2), normal_width, 0);
        polyline.emplace_back(Point(widths.size() * gap , -normal_width / 2), normal_width, 0);
        polyline.emplace_back(Point(-gap, -normal_width / 2), normal_width, 0);
        bool downward = true;
        for (int idx = 0; idx < widths.size(); idx++)
        {
            coord_t x = idx * gap;
            coord_t w = MM2INT(widths[widths.size() - 1 - idx]);
            for (bool first : {true, false})
            {
                coord_t y = (downward == first)? w / 2 : size - w / 2;
                polyline.emplace_back(Point(x, y), w, 0);
            }
            downward = !downward;
        }
        return result_polylines_per_index;
    }
};

} // namespace arachne
#endif // TEST_GEOMETRY_VARIABLE_WIDTH_GCODE_TESTER_H
