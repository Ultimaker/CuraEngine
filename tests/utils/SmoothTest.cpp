// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <numbers>

#include <range/v3/all.hpp>

#include <gtest/gtest.h>

#include "types/actions/smooth_fn.h"
#include "types/geometry/Point2LL.h"
#include "types/geometry/Polygon.h"

namespace curaengine
{
using namespace types::geometry;
using namespace types::actions;

TEST(SmoothTest, TestSmooth)
{
    // test isSmooth utility function
    smooth_fn smooth;
    const auto fluid_angle = 15.;
    const auto cos_fluid_angle = std::cos(fluid_angle * std::numbers::pi / 180.);

    {
        /*
         *
         *  a ------------- b
         *                  |
         *                  c --------------- d
         *
         */

        auto a = Point2LL{ 0, 0 };
        auto b = Point2LL{ 0, 100 };
        auto c = Point2LL{ 1, 100 };
        auto d = Point2LL{ 1, 200 };

        const auto is_smooth = smooth.isSmooth(a, b, c, d, cos_fluid_angle);
        EXPECT_EQ(is_smooth, false);
    }

    {
        /*
         *
         *  a ----------- b
         *                 \
         *                  c
         *                  |
         *                  |
         *                  |
         *                  d
         *
         */
        auto a = Point2LL{ 0, 0 };
        auto b = Point2LL{ 100, 0 };
        auto c = Point2LL{ 101, 1 };
        auto d = Point2LL{ 101, 101 };

        const auto is_smooth = smooth.isSmooth(a, b, c, d, cos_fluid_angle);
        EXPECT_EQ(is_smooth, true);
    }

    {
        /*
         *
         *  a ----------- b - c -------------d
         *
         */
        auto a = Point2LL{ 0, 0 };
        auto b = Point2LL{ 100, 0 };
        auto c = Point2LL{ 101, 0 };
        auto d = Point2LL{ 201, 0 };

        const auto is_smooth = smooth.isSmooth(a, b, c, d, cos_fluid_angle);
        EXPECT_EQ(is_smooth, true);
    }

    {
        /*
         *
         *  d ----------- c - b -------------a
         *
         */
        auto a = Point2LL{ 201, 0 };
        auto b = Point2LL{ 101, 0 };
        auto c = Point2LL{ 100, 0 };
        auto d = Point2LL{ 0, 0 };

        const auto is_smooth = smooth.isSmooth(a, b, c, d, cos_fluid_angle);
        EXPECT_EQ(is_smooth, true);
    }

    {
        /*
         *
         *
         *              c
         *               \
         *  a ----------- b
         *                 \
         *                  \
         *                   \
         *                    \
         *                     \
         *                      d
         *
         */
        auto a = Point2LL{ 0, 0 };
        auto b = Point2LL{ 100, 0 };
        auto c = Point2LL{ 99, -1 };
        auto d = Point2LL{ 199, 99 };

        const auto is_smooth = smooth.isSmooth(a, b, c, d, cos_fluid_angle);
        EXPECT_EQ(is_smooth, false);
    }

    {
        /*
         *
         *  d ----------- c
         *                 \
         *                  b
         *                   \
         *                    \
         *                     \
         *                      \
         *                       d
         *
         */
        auto a = Point2LL{ 0, 0 };
        auto b = Point2LL{ 100, 0 };
        auto c = Point2LL{ 101, 1 };
        auto d = Point2LL{ 201, 101 };

        const auto is_smooth = smooth.isSmooth(a, b, c, d, cos_fluid_angle);
        EXPECT_EQ(is_smooth, true);
    }

    {
        /*
         *
         *  a ----------- b - c
         *                    |
         *                    |
         *                    |
         *                    |
         *                    d
         *
         */
        Point2LL a = { 0, 0 };
        Point2LL b = { 100, 0 };
        Point2LL c = { 101, 0 };
        Point2LL d = { 101, 100 };

        const auto is_smooth = smooth.isSmooth(a, b, c, d, cos_fluid_angle);
        EXPECT_EQ(is_smooth, true);
    }

    {
        // real life example of a line that is clearly not smooth
        auto a = Point2LL{ 148451, 162177 };
        auto b = Point2LL{ 148854, 162229 };
        auto c = Point2LL{ 148866, 162244 };
        auto d = Point2LL{ 149772, 162297 };

        const auto is_smooth = smooth.isSmooth(a, b, c, d, cos_fluid_angle);
        EXPECT_EQ(is_smooth, false);
    };
}
} // namespace curaengine
