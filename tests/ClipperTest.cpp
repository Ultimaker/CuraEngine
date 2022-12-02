// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/Coord_t.h"
#include <cmath>
#include <gtest/gtest.h>
#include <polyclipping/clipper.hpp>

// #define TEST_INFILL_SVG_OUTPUT
#ifdef TEST_INFILL_SVG_OUTPUT
#include "utils/SVG.h"
#include "utils/polygon.h"
#include <cstdlib>
#endif // TEST_INFILL_SVG_OUTPUT

// NOLINTBEGIN(*-magic-numbers)
namespace cura
{
// This test currently always fails because there is a bug in clipper related to doing an intersection between a polygon and a polyline.
class DISABLED_ClipperTest : public testing::Test
{
public:
    using Paths = ClipperLib::Paths;
    using Path = ClipperLib::Path;
    using coord_t = ClipperLib::cInt;
    using Point = ClipperLib::IntPoint;

    std::vector<Paths> all_outlines;
    std::vector<Paths> all_polylines;

    DISABLED_ClipperTest()
    {
    }

    void SetUp() override
    {
        {
            all_outlines.emplace_back();
            all_outlines.back().emplace_back();
            Path& outline = all_outlines.back().back();
            outline.emplace_back(1100, 500);
            outline.emplace_back(500, 500);
            outline.emplace_back(500, 400);
            outline.emplace_back(1100, 400);

            all_polylines.emplace_back();
            all_polylines.back().emplace_back();
            Path& polyline = all_polylines.back().back();
            polyline.emplace_back(1075, 425);
            polyline.emplace_back(672, 425);
            polyline.emplace_back(651, 425);
            polyline.emplace_back(595, 424);
            polyline.emplace_back(617, 437);
        }
        {
            all_outlines.emplace_back();
            all_outlines.back().emplace_back();
            Path& outline = all_outlines.back().back();
            outline.emplace_back(1000, 1100);
            outline.emplace_back(300, 1100);
            outline.emplace_back(200, 1000);
            outline.emplace_back(1000, 1000);

            all_polylines.emplace_back();
            all_polylines.back().emplace_back();
            Path& polyline = all_polylines.back().back();
            polyline.emplace_back(992, 1050);
            polyline.emplace_back(617, 1049);
            polyline.emplace_back(660, 1074);
            polyline.emplace_back(617, 1049);
            polyline.emplace_back(582, 1049);
            polyline.emplace_back(538, 1074);
        }
    }

    void TearDown() override
    {
    }

    static Paths intersectPolylines(const Paths& outlines, const Paths& polylines);

    static coord_t sq(coord_t i)
    {
        return i * i;
    };

    static coord_t calculateLength(const Paths& polylines);

    static void outputSVG(const Paths& outlines, const Paths& polylines, const Paths& intersected, const char* filename);
};

DISABLED_ClipperTest::Paths DISABLED_ClipperTest::intersectPolylines(const Paths& outlines, const Paths& polylines)
{
    ClipperLib::PolyTree result_tree;
    ClipperLib::Clipper clipper(0);
    clipper.AddPaths(polylines, ClipperLib::ptSubject, false);
    clipper.AddPaths(outlines, ClipperLib::ptClip, true);
    clipper.Execute(ClipperLib::ctIntersection, result_tree);
    Paths result;
    ClipperLib::PolyTreeToPaths(result_tree, result);
    return result;
}

DISABLED_ClipperTest::coord_t DISABLED_ClipperTest::calculateLength(const Paths& polylines)
{
    coord_t polyline_length = 0;
    for (const Path& path : polylines)
    {
        const Point* last = nullptr;
        for (const Point& p : path)
        {
            if (last)
            {
                polyline_length += std::sqrt(sq(p.X - last->X) + sq(p.Y - last->Y));
            }
            last = &p;
        }
    }
    return polyline_length;
}


TEST_F(DISABLED_ClipperTest, PolylinesTest1)
{
    Paths intersected = intersectPolylines(all_outlines[0], all_polylines[0]);
    outputSVG(all_outlines[0], all_polylines[0], intersected, "/tmp/clipper_test_0");
    ASSERT_EQ(calculateLength(intersected), calculateLength(all_polylines[0]));
}

TEST_F(DISABLED_ClipperTest, PolylinesTest2)
{
    Paths intersected = intersectPolylines(all_outlines[1], all_polylines[1]);
    outputSVG(all_outlines[1], all_polylines[1], intersected, "/tmp/clipper_test_1");
    ASSERT_EQ(calculateLength(intersected), calculateLength(all_polylines[1]));
}


void DISABLED_ClipperTest::outputSVG(const Paths& outlines, const Paths& polylines, const Paths& intersected, const char* filename)
{
#ifdef TEST_INFILL_SVG_OUTPUT
    Polygons outs;
    outs.set(outlines);
    Polygons lines;
    lines.set(polylines);
    Polygons lines_after;
    lines_after.set(intersected);
    SVG svg(filename, AABB(outs));
    svg.writePolygons(outs, SVG::Color::BLACK);
    svg.nextLayer();
    svg.writePolylines(lines_after, SVG::Color::RED);
    svg.nextLayer();
    svg.writePolylines(lines, SVG::Color::GREEN);
#endif // TEST_INFILL_SVG_OUTPUT
}


} // namespace cura
// NOLINTEND(*-magic-numbers)