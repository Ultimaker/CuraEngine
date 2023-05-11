// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <gtest/gtest.h>

#include "utils/concepts/geometry.h"
#include "utils/geometry/point_container.h"
#include "utils/views/segments.h"

namespace cura
{

TEST(ViewTest, Polyline)
{
    auto polyline = geometry::polyline{ { 0, 0 }, { 1, 1 }, { 2, 2 } };
    static_assert(! is_closed<decltype(polyline)>::value);

    ASSERT_EQ(polyline.size(), 3);

    for (auto p : polyline)
    {
        ASSERT_EQ(p.X, p.Y);
    }
}

TEST(ViewTest, Polygon)
{
    auto polygon = geometry::polygon_outer{ { 0, 0 }, { 1, 1 }, { 2, 2 } };
    static_assert(is_closed<decltype(polygon)>::value);
    static_assert(is_clockwise<decltype(polygon)>::value);

    ASSERT_EQ(polygon.size(), 3);

    for (auto p : polygon)
    {
        ASSERT_EQ(p.X, p.Y);
    }
}

TEST(ViewTest, SegmentsViewPolyline)
{
    auto polyline = geometry::polyline{ { 0, 0 }, { 1, 1 }, { 2, 2 } };
    static_assert(concepts::polyline<decltype(polyline)>);
    ASSERT_EQ(polyline.size(), 3);

    auto polyline_view = polyline | views::segments;
    ASSERT_EQ(polyline_view.size(), 3);

    for (auto p : polyline_view)
    {
        ASSERT_EQ(p.X, p.Y);
    }
}

TEST(ViewTest, SegmentsViewPolygon)
{
    auto polygon = geometry::polyline({ { 0, 0 }, { 1, 1 }, { 2, 2 } });
    static_assert(concepts::polyline<decltype(polygon)>);
    ASSERT_EQ(polygon.size(), 3);

    auto polygon_view = polygon | views::segments;
    ASSERT_EQ(polygon_view.size(), 4);

    for (auto p : polygon_view)
    {
        ASSERT_EQ(p[0].X, p[0].Y);
    }
}

} // namespace cura