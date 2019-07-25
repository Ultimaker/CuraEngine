//Copyright (c) 2019 Ultimaker B.V.


#ifndef TEST_MICROSTRUCTURE_H
#define TEST_MICROSTRUCTURE_H

#include <utility>

#include "../utils/IntPoint.h"
#include "../utils/logoutput.h"
#include "../utils/polygon.h"
#include "../utils/ExtrusionSegment.h"

namespace arachne
{

class Microstructure
{
public:
    Microstructure()
    {}
    Polygons squareGrid(Point grid_size, Point cell_size)
    {
        Polygons ret;
        for (int x = 0; x < grid_size.X; x++)
            for (int y = 0; y < grid_size.Y; y++)
            {
                Point min(cell_size.X * x, cell_size.Y * y);
                Point max = min + cell_size;
                if (y < grid_size.Y - 1) ret.add(generateSegment(min, Point(min.X, max.Y)));
                if (x < grid_size.X - 1) ret.add(generateSegment(min, Point(max.X, min.Y)));
            }
        ret = ret.execute(ClipperLib::pftPositive);
        return ret;
    }
private:
    coord_t step_size = MM2INT(0.2);
    Polygons generateSegment(Point from, Point to)
    {
        Polygons ret;
        Point vec = to - from;
        coord_t length = vSize(vec);
        coord_t step_count = std::max(coord_t(0), (length + step_size / 2) / step_size);
        ExtrusionJunction prev(from, getW(from), 0);
        for (coord_t step = 0; step < step_count; step++)
        {
            Point p = from + vec * (step + 1) / step_count;
            ExtrusionJunction here(p, getW(p), 0);
            ExtrusionSegment s(prev, here, true);
            Polygons area = s.toPolygons(false);
            ret.add(area);
            prev = here;
        }
        return ret;
    }
    coord_t getW(Point p)
    {
        Point diff = p - Point(5000, 2000);
        diff.X = diff.X / 2 + diff.Y / 4;
        diff = diff / 2;
        float d = INT2MM(vSize(diff) / 3) / 4;
        return std::max(350.0, 2020 * 1.0 / (1.0 + d * d));
    }
};

} // namespace arachne
#endif // TEST_MICROSTRUCTURE_H
