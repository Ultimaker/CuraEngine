// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <gtest/gtest.h>
#include <range/v3/view/zip.hpp>
#include <spdlog/spdlog.h>

#include "utils/types/geometry.h"
#include "utils/views/segments.h"
#include "utils/views/simplify.h"
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

TEST(ViewTest, SimplifyViewPolygon)
{
    auto polygon = geometry::closed_path({ { 0, 0 }, { 1, 1 }, { 2, 2 } });
    auto polygon_view = polygon | views::simplify(500) | ranges::views::all;
}

} // namespace cura