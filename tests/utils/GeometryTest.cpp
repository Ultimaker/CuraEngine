// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <gtest/gtest.h>
#include <range/v3/view/zip.hpp>

#include "geometry/point_container.h"
#include "utils/polygon.h"
#include "utils/types/geometry.h"
#include "utils/views/segments.h"


namespace cura
{
TEST(GeometryTest, open_path)
{
    auto polyline = geometry::open_path{ { 0, 0 }, { 1, 1 }, { 2, 2 } };
    static_assert(! utils::closed_path<decltype(polyline)>);
    static_assert(utils::open_path<decltype(polyline)>);

    ASSERT_EQ(polyline.size(), 3);

    for (auto p : polyline)
    {
        ASSERT_EQ(p.X, p.Y);
    }
}

TEST(GeometryTest, closed_path)
{
    auto polygon = geometry::closed_path{ { 0, 0 }, { 1, 1 }, { 2, 2 } };
    static_assert(utils::closed_path<decltype(polygon)>);
    static_assert(! utils::open_path<decltype(polygon)>);

    ASSERT_EQ(polygon.size(), 3);

    for (auto p : polygon)
    {
        ASSERT_EQ(p.X, p.Y);
    }
}


TEST(GeometryTest, closed_path_from_polygon)
{
    Polygon polygon;
    polygon.emplace_back(0, 0);
    polygon.emplace_back(100, 0);
    polygon.emplace_back(100, 100);
    polygon.emplace_back(0, 100);

    auto closed_path = geometry::closed_path{ polygon };

    auto polygon_view = closed_path | views::segments;
    auto expected = std::vector<std::pair<Point, Point>>{
        { { 0, 0 }, { 100, 0 } },
        { { 100, 0 }, { 100, 100 } },
        { { 100, 100 }, { 0, 100 } },
        { { 0, 100 }, { 0, 0 } }
    };

    for (const auto& [val, exp] : ranges::views::zip(polygon_view, expected))
    {
        ASSERT_EQ(val.first, exp.first);
        ASSERT_EQ(val.second, exp.second);
    }
}

TEST(GeometryTest, ranged_paths_from_Polygons)
{
    Polygon polygon_inner;
    polygon_inner.emplace_back(10, 10);
    polygon_inner.emplace_back(90, 10);
    polygon_inner.emplace_back(90, 90);
    polygon_inner.emplace_back(10, 90);

    Polygon polygon_outer;
    polygon_outer.emplace_back(0, 0);
    polygon_outer.emplace_back(100, 0);
    polygon_outer.emplace_back(100, 100);
    polygon_outer.emplace_back(0, 100);

    Polygons polygons;
    polygons.emplace_back(polygon_outer);
    polygons.emplace_back(polygon_inner);

    auto polygon = geometry::ranged_paths{ polygons };

    auto expected = std::vector<std::vector<std::pair<Point, Point>>>{
        { { 0, 0 }, { 100, 0 }, { 100, 100 }, { 0, 100 }, { 0, 0 } },
        { { 10, 10 }, { 90, 10 }, { 90, 90 }, { 10, 90 }, { 10, 10 } }
    };

    for (const auto& [path, exp_path] : ranges::views::zip(polygon, expected))
    {
        for (const auto& [val, exp] : ranges::views::zip(path, exp_path))
        {
            ASSERT_EQ(val.X, exp.first);
            ASSERT_EQ(val.Y, exp.second);
        }
    }
}

} // namespace cura