// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <gtest/gtest.h>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/zip.hpp>
#include <spdlog/spdlog.h>

#include "utils/types/geometry.h"
#include "utils/views/segments.h"
#include "utils/views/simplify.h"
#include "utils/views/subdivide.h"
#include "geometry/point_container.h"

namespace cura
{

TEST(ViewTest, SegmentsViewPolyline)
{
    auto polyline = geometry::open_path{ { 0, 0 }, { 1, 1 }, { 2, 2 } };

    auto polyline_view = polyline | views::segments;
    auto expected = std::vector<std::pair<Point, Point>>{
        { { 0, 0 }, { 1, 1 } },
        { { 1, 1 }, { 2, 2 } }
    };

    for (const auto& [val, exp] : ranges::views::zip(polyline_view, expected))
    {
        ASSERT_EQ(val.first, exp.first);
        ASSERT_EQ(val.second, exp.second);
    }
}

TEST(ViewTest, SegmentsViewPolygon)
{
    auto polygon = geometry::closed_path({ { 0, 0 }, { 1, 1 }, { 2, 2 } });

    auto polygon_view = polygon | views::segments;
    auto expected = std::vector<std::pair<Point, Point>>{
        { { 0, 0 }, { 1, 1 } },
        { { 1, 1 }, { 2, 2 } },
        { { 2, 2 }, { 0, 0 } }
    };

    for (const auto& [val, exp] : ranges::views::zip(polygon_view, expected))
    {
        ASSERT_EQ(val.first, exp.first);
        ASSERT_EQ(val.second, exp.second);
    }
}

TEST(ViewTest, SudividePolygon)
{
    auto polygon = geometry::closed_path({ { 0, 0 }, { 200, 0 }, { 0, 200 } });

    auto polygon_res = polygon | views::segments | views::subdivide<views::subdivide_stops::Mid> | ranges::to<std::vector>;
    auto expected = std::vector<std::pair<Point, Point>>{
        { {   0,   0 }, { 100,   0 } },
        { { 100,   0 }, { 200,   0 } },
        { { 200,   0 }, { 100, 100 } },
        { { 100, 100 }, {   0, 200 } },
        { {   0, 200 }, {   0, 100 } },
        { {   0, 100 }, {   0,   0 } }
    };

    ASSERT_EQ(polygon_res.size(), expected.size());
    for (const auto& [val, exp] : ranges::views::zip(polygon_res, expected))
    {
        ASSERT_EQ(val.first, exp.first);
        ASSERT_EQ(val.second, exp.second);
    }
}

TEST(ViewTest, SimplifyViewPolygon)
{
    auto polygon = geometry::closed_path{ { 0, 100 }, { 0, 0 }, { 10, 5 }, { 100, 0 }, { 100, 100 }};
    auto polygon_view = polygon | views::simplify(5) | ranges::views::all;
    for (const auto& pt : polygon_view)
    {
        spdlog::info("{},{}", pt.X, pt.Y);
    }
}

TEST(ViewTest, simplified_paths)
{
    auto polygon = std::vector{ geometry::closed_path{ { 0, 0 }, { 100, 0 }, { 100, 100 }, { 0, 100 }, { 0, 0 } }, { { 10, 10 }, { 90, 10 }, { 90, 90 }, { 10, 90 }, { 10, 10 } } };
    auto simplified = polygon | views::simplify(1) | ranges::views::all;

    for (const auto& path : simplified)
    {
        for (const auto& pt : path)
        {
            spdlog::info("{},{}", pt.X, pt.Y);
        }
    }

}

} // namespace cura
