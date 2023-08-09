// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <gtest/gtest.h>

#include <set>

#include <range/v3/all.hpp>

#include "utils/IntPoint.h"
#include "utils/actions/smooth.h"
#include "utils/polygon.h"

namespace cura
{

TEST(SmoothTest, TestSmooth)
{
    // test isSmooth utility function
    cura::actions::smooth_fn smooth;
    const auto FLUID_ANGLE = 5.;
    const auto COS_FLUID_ANGLE = std::cos(FLUID_ANGLE * M_PI / 180.);

    {
        /*
         *
         *  A ------------- B
         *                  |
         *                  C --------------- D
         *
         */

        auto A = cura::Point { 0, 0 };
        auto B = cura::Point { 0, 100 };
        auto C = cura::Point { 1, 100 };
        auto D = cura::Point { 1, 200 };

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
        auto A = cura::Point { 0, 0 };
        auto B = cura::Point { 100, 0 };
        auto C = cura::Point { 101, 1 };
        auto D = cura::Point { 101, 101 };

        const auto is_smooth = smooth.isSmooth(A, B, C, D, COS_FLUID_ANGLE);
        EXPECT_EQ(is_smooth, true);
    }

    {
        /*
         *
         *  A ----------- B - C -------------D
         *
         */
        auto A = cura::Point { 0, 0 };
        auto B = cura::Point { 100, 0 };
        auto C = cura::Point { 101, 0 };
        auto D = cura::Point { 201, 0 };

        const auto is_smooth = smooth.isSmooth(A, B, C, D, COS_FLUID_ANGLE);
        EXPECT_EQ(is_smooth, true);
    }

    {
        /*
         *
         *  D ----------- C - B -------------A
         *
         */
        auto A = cura::Point { 201, 0 };
        auto B = cura::Point { 101, 0 };
        auto C = cura::Point { 100, 0 };
        auto D = cura::Point { 0, 0 };

        const auto is_smooth = smooth.isSmooth(A, B, C, D, COS_FLUID_ANGLE);
        EXPECT_EQ(is_smooth, true);
    }

    {
        /*
         *
         *
         *              B
         *               \
         *  D ----------- C
         *                 \
         *                  \
         *                   \
         *                    \
         *                     \
         *                      D
         *
         */
        auto A = cura::Point { 0, 0 };
        auto B = cura::Point { 100, 0 };
        auto C = cura::Point { 99, -1 };
        auto D = cura::Point { 199, 99 };

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
        auto A = cura::Point { 0, 0 };
        auto B = cura::Point { 100, 0 };
        auto C = cura::Point { 101, 1 };
        auto D = cura::Point { 201, 101 };

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
        cura::Point A = { 0, 0 };
        cura::Point B = { 100, 0 };
        cura::Point C = { 101, 0 };
        cura::Point D = { 101, 100 };

        const auto is_smooth = smooth.isSmooth(A, B, C, D, COS_FLUID_ANGLE);
        EXPECT_EQ(is_smooth, true);
    }

}
} // namespace cura