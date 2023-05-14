// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <gtest/gtest.h>
#include <range/v3/view/zip.hpp>
#include <spdlog/spdlog.h>

#include "utils/concepts/geometry.h"
#include "utils/geometry/point_container.h"
#include "utils/views/segments.h"

namespace cura
{

TEST(ViewTest, Polyline)
{
    auto polyline = geometry::polyline{ { 0, 0 }, { 1, 1 }, { 2, 2 } };
    static_assert(is_open_container_v<decltype(polyline)>);

    ASSERT_EQ(polyline.size(), 3);

    for (auto p : polyline)
    {
        ASSERT_EQ(p.X, p.Y);
    }
}

TEST(ViewTest, Polygon)
{
    auto polygon = geometry::polygon_outer{ { 0, 0 }, { 1, 1 }, { 2, 2 } };
    static_assert(is_closed_container_v<decltype(polygon)>);

    ASSERT_EQ(polygon.size(), 3);

    for (auto p : polygon)
    {
        ASSERT_EQ(p.X, p.Y);
    }
}

TEST(ViewTest, SegmentsViewPolyline)
{
    auto polyline = geometry::polyline{ { 0, 0 }, { 1, 1 }, { 2, 2 } };

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
    auto polygon = geometry::polygon_outer({ { 0, 0 }, { 1, 1 }, { 2, 2 } });

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

} // namespace cura