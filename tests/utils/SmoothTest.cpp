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
    const auto FLUID_ANGLE = 15.;
    const auto COS_FLUID_ANGLE = std::cos(FLUID_ANGLE * std::numbers::pi / 180.);

    {
        /*
         *
         *  A ------------- B
         *                  |
         *                  C --------------- D
         *
         */

        auto A = Point2LL{ 0, 0 };
        auto B = Point2LL{ 0, 100 };
        auto C = Point2LL{ 1, 100 };
        auto D = Point2LL{ 1, 200 };

        const auto is_smooth = smooth.isSmooth(A, B, C, D, COS_FLUID_ANGLE);
        EXPECT_EQ(is_smooth, false);
    }

    {
        /*
         *
         *  A ----------- B
         *                 \
         *                  C
         *                  |
         *                  |
         *                  |
         *                  D
         *
         */
        auto A = Point2LL{ 0, 0 };
        auto B = Point2LL{ 100, 0 };
        auto C = Point2LL{ 101, 1 };
        auto D = Point2LL{ 101, 101 };

        const auto is_smooth = smooth.isSmooth(A, B, C, D, COS_FLUID_ANGLE);
        EXPECT_EQ(is_smooth, true);
    }

    {
        /*
         *
         *  A ----------- B - C -------------D
         *
         */
        auto A = Point2LL{ 0, 0 };
        auto B = Point2LL{ 100, 0 };
        auto C = Point2LL{ 101, 0 };
        auto D = Point2LL{ 201, 0 };

        const auto is_smooth = smooth.isSmooth(A, B, C, D, COS_FLUID_ANGLE);
        EXPECT_EQ(is_smooth, true);
    }

    {
        /*
         *
         *  D ----------- C - B -------------A
         *
         */
        auto A = Point2LL{ 201, 0 };
        auto B = Point2LL{ 101, 0 };
        auto C = Point2LL{ 100, 0 };
        auto D = Point2LL{ 0, 0 };

        const auto is_smooth = smooth.isSmooth(A, B, C, D, COS_FLUID_ANGLE);
        EXPECT_EQ(is_smooth, true);
    }

    {
        /*
         *
         *
         *              C
         *               \
         *  A ----------- B
         *                 \
         *                  \
         *                   \
         *                    \
         *                     \
         *                      D
         *
         */
        auto A = Point2LL{ 0, 0 };
        auto B = Point2LL{ 100, 0 };
        auto C = Point2LL{ 99, -1 };
        auto D = Point2LL{ 199, 99 };

        const auto is_smooth = smooth.isSmooth(A, B, C, D, COS_FLUID_ANGLE);
        EXPECT_EQ(is_smooth, false);
    }

    {
        /*
         *
         *  D ----------- C
         *                 \
         *                  B
         *                   \
         *                    \
         *                     \
         *                      \
         *                       D
         *
         */
        auto A = Point2LL{ 0, 0 };
        auto B = Point2LL{ 100, 0 };
        auto C = Point2LL{ 101, 1 };
        auto D = Point2LL{ 201, 101 };

        const auto is_smooth = smooth.isSmooth(A, B, C, D, COS_FLUID_ANGLE);
        EXPECT_EQ(is_smooth, true);
    }

    {
        /*
         *
         *  A ----------- B - C
         *                    |
         *                    |
         *                    |
         *                    |
         *                    D
         *
         */
        Point2LL A = { 0, 0 };
        Point2LL B = { 100, 0 };
        Point2LL C = { 101, 0 };
        Point2LL D = { 101, 100 };

        const auto is_smooth = smooth.isSmooth(A, B, C, D, COS_FLUID_ANGLE);
        EXPECT_EQ(is_smooth, true);
    }

    {
        // real life example of a line that is clearly not smooth
        auto A = Point2LL{ 148451, 162177 };
        auto B = Point2LL{ 148854, 162229 };
        auto C = Point2LL{ 148866, 162244 };
        auto D = Point2LL{ 149772, 162297 };

        const auto is_smooth = smooth.isSmooth(A, B, C, D, COS_FLUID_ANGLE);
        EXPECT_EQ(is_smooth, false);
    };
}
} // namespace curaengine
