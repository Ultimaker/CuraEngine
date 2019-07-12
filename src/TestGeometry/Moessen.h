//Copyright (c) 2019 Ultimaker B.V.


#ifndef TEST_GEOMETRY_MOESSEN_H
#define TEST_GEOMETRY_MOESSEN_H

#include <utility>

#include "../utils/IntPoint.h"
#include "../utils/logoutput.h"
#include "../utils/polygon.h"

namespace arachne
{

class MoessenTests
{
public:
    static Polygons generateCircles(Point grid_size, coord_t min_r, coord_t max_r, coord_t skip_r, coord_t angle_count = 50)
    {
        Polygons ret;
        PolygonRef square = ret.newPoly();
        square.emplace_back(0, 0);
        square.emplace_back(0, grid_size.Y * skip_r * 2);
        square.emplace_back(grid_size * skip_r * 2);
        square.emplace_back(grid_size.X * skip_r * 2, 0);

        for (int x = 0; x < grid_size.X; x++)
            for (int y = 0; y < grid_size.Y; y++)
            {
                PolygonRef circle = ret.newPoly();
                coord_t r = min_r + rand() % (max_r - min_r);
                for (coord_t v = 0; v < angle_count; v++)
                {
                    float a = 2.0 * M_PI / angle_count * v;
                    circle.emplace_back(Point(x * 2 + 1, y * 2 + 1) * skip_r + Point(r * cos(a), r * sin(a)));
                }
            }
        return ret;
    }

    static Polygons generateTriangles(Point grid_size, coord_t min_r, coord_t max_r, coord_t skip_r)
    {
        Polygons ret;
        PolygonRef square = ret.newPoly();
        coord_t side_length = skip_r * 2 * sqrt(3.0);
        coord_t height = skip_r * 3;
        square.emplace_back(0, 0);
        square.emplace_back(grid_size.Y * side_length / 2, grid_size.Y * height);
        square.emplace_back(grid_size.Y * side_length / 2 + grid_size.X * side_length / 2, grid_size.Y * height);
        square.emplace_back(grid_size.X * side_length / 2, 0);
        
        
        for (int x = 0; x < grid_size.X; x++)
            for (int y = 0; y < grid_size.Y; y++)
            {
                PolygonRef circle = ret.newPoly();
                coord_t r = (max_r == min_r)? min_r : min_r + rand() % (max_r - min_r);
                r *= 2;
                for (coord_t v = 0; v < 3; v++)
                {
                    float a = 2.0 * M_PI / 3 * v + M_PI / 2;
                    Point mid = Point(x * side_length / 2 + side_length * y / 2 + side_length / 2, y * height + skip_r);
                    if (x % 2 == 1)
                    {
                        mid += Point(0, skip_r);
                        a += M_PI;
                    }
                    circle.emplace_back(mid + Point(r * cos(a), r * sin(a)));
                }
            }
        return ret;
    }
};

} // namespace arachne
#endif // TEST_GEOMETRY_MOESSEN_H
