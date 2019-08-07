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
    static std::vector<std::vector<std::vector<ExtrusionJunction>>> zigzag(coord_t gap = MM2INT(2.0), coord_t size = MM2INT(50))
    {
        std::vector<std::vector<std::vector<ExtrusionJunction>>> result_polylines_per_index;
        result_polylines_per_index.resize(1);
        result_polylines_per_index[0].resize(1);
        std::vector<ExtrusionJunction>& polyline = result_polylines_per_index[0][0];
        
        coord_t normal_width = 400;
        std::vector<float> widths({0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4});
        /*
        polyline.emplace_back(Point(-gap*2, -gap*2), normal_width, 0);
        polyline.emplace_back(Point(-gap*2, size + gap*2), normal_width, 0);
        polyline.emplace_back(Point((widths.size()+1) * gap, size + gap*2), normal_width, 0);
        polyline.emplace_back(Point((widths.size()+1) * gap, -gap*2), normal_width, 0);
        polyline.emplace_back(Point(-gap*2 + normal_width, -gap*2), normal_width, 0);
        
        polyline.emplace_back(Point(-gap, -gap), normal_width, 0);
        polyline.emplace_back(Point(-gap, size + gap), normal_width, 0);
        polyline.emplace_back(Point(widths.size() * gap, size + gap), normal_width, 0);
        polyline.emplace_back(Point(widths.size() * gap, -gap), normal_width, 0);
        polyline.emplace_back(Point(-gap + normal_width, -gap), normal_width, 0);
        */
        bool downward = true;
        for (int idx = 0; idx < widths.size(); idx++)
        {
            coord_t x = idx * gap;
            for (bool first : {true, false})
            {
                coord_t y = (downward == first)? 0 : size;
                polyline.emplace_back(Point(x, y), MM2INT(widths[idx]), 0);
            }
            downward = !downward;
        }
        return result_polylines_per_index;
    }
};

} // namespace arachne
#endif // TEST_GEOMETRY_VARIABLE_WIDTH_GCODE_TESTER_H
