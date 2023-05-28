// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <gtest/gtest.h>

#include "geometry/point_container.h"
#include "utils/polygon.h"
#include "utils/types/geometry.h"


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

TEST(ViewTest, Polygon)
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


} // namespace cura