//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include "../src/utils/polygon.h"

// #define TEST_INFILL_SVG_OUTPUT
#ifdef TEST_INFILL_SVG_OUTPUT
#include <cstdlib>
#include "../src/utils/SVG.h"
#endif //TEST_INFILL_SVG_OUTPUT

namespace cura
{

class ClipperTest : public testing::Test
{
public:


    ClipperTest()
    {
    }

    void SetUp()
    {
    }

    void TearDown()
    {
    }
};

TEST_F(ClipperTest, PolylinesTest)
{

    auto sq = [](int i) { return i*i; };
    
    ClipperLib::Paths outlines;
    {
        outlines.emplace_back();
        ClipperLib::Path& outline = outlines.back();
        outline.emplace_back(1100, 500);
        outline.emplace_back(500, 500);
        outline.emplace_back(500, 400);
        outline.emplace_back(1100, 400);
        
//         outline.emplace_back(1000, 1100);
//         outline.emplace_back(300, 1100);
//         outline.emplace_back(200, 1000);
//         outline.emplace_back(1000, 1000);
    }
    
    ClipperLib::Paths polylines;
    {
        polylines.emplace_back();
        ClipperLib::Path& polyline = polylines.back();
        polyline.emplace_back(1075, 425);
        polyline.emplace_back(672, 425 );
        polyline.emplace_back(651, 425 );
        polyline.emplace_back(595, 424 );
        polyline.emplace_back(617, 437 );
        // old ^
        
        
//         polyline.emplace_back(992, 1050);
//         polyline.emplace_back(617, 1049);
//         polyline.emplace_back(660, 1074);
//         polyline.emplace_back(617, 1049);
//         polyline.emplace_back(582, 1049);
//         polyline.emplace_back(538, 1074);
    }
    ClipperLib::PolyTree result_tree;
    ClipperLib::Clipper clipper(0);
    clipper.AddPaths(polylines, ClipperLib::ptSubject, false);
    clipper.AddPaths(outlines, ClipperLib::ptClip, true);
    clipper.Execute(ClipperLib::ctIntersection, result_tree);
    ClipperLib::Paths result;
    ClipperLib::PolyTreeToPaths(result_tree, result);

    coord_t original_polyline_length = 0;
    for (ClipperLib::Path path : polylines)
    {
        Point* last = nullptr;
        for (Point& p : path)
        {
            if (last)
            {
                original_polyline_length += std::sqrt(sq(p.X - last->X) + sq(p.Y - last->Y));
            }
            last = &p;
        }
    }

    coord_t intersected_polyline_length = 0;
    for (ClipperLib::Path path : result)
    {
        Point* last = nullptr;
        for (Point& p : path)
        {
            if (last)
            {
                intersected_polyline_length += std::sqrt(sq(p.X - last->X) + sq(p.Y - last->Y));
            }
            last = &p;
        }
    }

    
    
#ifdef TEST_INFILL_SVG_OUTPUT
    {
        Polygons outs;
        outs.paths = outlines;
        Polygons lines;
        lines.paths = polylines;
        Polygons lines_after;
        lines_after.paths = result;
        SVG svg("/tmp/problem.svg", AABB(outs));
        svg.writePolygons(outs, SVG::Color::BLACK);
        svg.nextLayer();
        svg.writePolylines(lines_after, SVG::Color::RED);
        svg.nextLayer();
        svg.writePolylines(lines, SVG::Color::GREEN);
    }
#endif // TEST_INFILL_SVG_OUTPUT
    
    
    assert(intersected_polyline_length == original_polyline_length);
}


} //namespace cura
