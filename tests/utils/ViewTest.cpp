// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <gtest/gtest.h>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/cache1.hpp>
#include <range/v3/view/zip.hpp>
#include <spdlog/spdlog.h>
#include <vector>

#include "geometry/point_container.h"
#include "utils/types/geometry.h"
#include "utils/views/segments.h"
#include "utils/views/simplify.h"
#include "utils/views/subdivide.h"

namespace cura
{

TEST(ViewTest, SegmentsViewPolyline)
{
    auto polyline = geometry::open_path{ { 0, 0 }, { 1, 1 }, { 2, 2 } };

    auto polyline_view = polyline | views::segments;
    auto expected = std::vector<std::pair<Point, Point>>{ { { 0, 0 }, { 1, 1 } }, { { 1, 1 }, { 2, 2 } } };

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
    auto expected = std::vector<std::pair<Point, Point>>{ { { 0, 0 }, { 1, 1 } }, { { 1, 1 }, { 2, 2 } }, { { 2, 2 }, { 0, 0 } } };

    for (const auto& [val, exp] : ranges::views::zip(polygon_view, expected))
    {
        ASSERT_EQ(val.first, exp.first);
        ASSERT_EQ(val.second, exp.second);
    }
}

TEST(ViewTest, SudividePolygon)
{
    auto polygon =
        {
            geometry::closed_path({ { 0, 0 }, { 200, 0 }, { 0, 200 } }),
            geometry::closed_path({ { 0, 0 }, { 200, 0 }, { 0, 200 } }),
        };

    auto polygon_res = polygon | views::segments | views::subdivide<views::subdivide_stops::Mid>(0) | ranges::to<std::vector>;
    auto expected =
        std::vector<std::vector<std::pair<Point, Point>>>
        {
            { { { 0, 0 }, { 100, 0 } }, { { 100, 0 }, { 200, 0 } }, { { 200, 0 }, { 100, 100 } }, { { 100, 100 }, { 0, 200 } }, { { 0, 200 }, { 0, 100 } }, { { 0, 100 }, { 0, 0 } } },
            { { { 0, 0 }, { 100, 0 } }, { { 100, 0 }, { 200, 0 } }, { { 200, 0 }, { 100, 100 } }, { { 100, 100 }, { 0, 200 } }, { { 0, 200 }, { 0, 100 } }, { { 0, 100 }, { 0, 0 } } }
        };

    ASSERT_EQ(polygon_res.size(), expected.size());
    for (const auto& [val_, exp_] : ranges::views::zip(polygon_res, expected))
    {
        ASSERT_EQ(val_.size(), exp_.size());
        for (const auto& [val, exp] : ranges::views::zip(val_, exp_))
        {
            ASSERT_EQ(val.first, exp.first);
            ASSERT_EQ(val.second, exp.second);
        }
    }
}

TEST(ViewTest, SimplifyViewPolygon)
{
    auto polygon = geometry::closed_path{ { 0, 0 }, { 10, 5 }, { 100, 0 }, { 100, 100 }, { 0, 100 } };
    auto polygon_view = polygon | views::simplify(5) | ranges::views::all;
    auto expected = std::vector<Point>{ { 100, 100 }, { 0, 100 }, { 0, 0 }, { 100, 0 } };

    for (const auto& pt : ranges::views::zip(polygon_view, expected))
    {
        ASSERT_EQ(pt.first, pt.second);
    }
}

TEST(ViewTest, SimplifyViewPolyline)
{
    auto polyline = geometry::open_path{ { 0, 0 }, { 10, 5 }, { 100, 0 }, { 100, 100 }, { 0, 100 } };
    auto polyline_view = polyline | views::simplify(5) | ranges::views::all;
    auto expected = std::vector<Point>{ { 0, 0 }, { 100, 0 }, { 100, 100 }, { 0, 100 } };

    for (const auto& pt : ranges::views::zip(polyline_view, expected))
    {
        ASSERT_EQ(pt.first, pt.second);
    }
}

TEST(ViewTest, simplified_paths)
{
    auto polygon = std::vector{ geometry::closed_path{ { 0, 0 }, { 10, 5 }, { 100, 0 }, { 100, 100 }, { 0, 100 }, { 0, 0 } }, { { 10, 10 }, { 90, 10 }, { 90, 90 }, { 10, 90 }, { 10, 10 } } };
    auto simplified = polygon | views::simplify(5) | ranges::views::all;

    auto expected = std::vector{ geometry::closed_path{ { 100, 100 }, { 0, 100 }, { 0, 0 }, { 100, 0 } }, { { 90, 90 }, { 10, 90 }, { 10, 10 }, { 90, 10 } } };

    for (const auto& [path, exp_path] : ranges::views::zip(simplified, expected))
    {
        for (const auto& pt : ranges::views::zip(path, exp_path))
        {
            ASSERT_EQ(pt.first, pt.second);
        }
    }
}

} // namespace cura
