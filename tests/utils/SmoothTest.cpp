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
    // TODO: Write some actual tests
    Polygon poly;
    poly.poly = std::vector<cura::Point>{ { -137, 188 },  { 1910, 540 },  { 3820, 540 },  { 3850, 640 },  { 5040, 780 },  { 5660, 2800 }, { 5420, 2720 }, { 5500, 2850 },
                                          { 5530, 2970 }, { 5290, 3450 }, { 1610, 4030 }, { 1090, 3220 }, { 1060, 3210 }, { 1010, 3210 }, { 970, 3220 },  { -740, 3940 } };
    auto smoother = cura::actions::smooth(2000, 10.0);
    Polygon smoothed;
    smoothed.poly = smoother(poly.poly);

    std::vector<cura::Point> expected{ { -137, 188 },  { 1910, 540 },  { 2820, 540 },  { 3850, 640 },  { 5040, 780 },  { 5660, 2800 }, { 5420, 2720 }, { 5500, 2850 },
                                       { 5530, 2970 }, { 5290, 3450 }, { 1610, 4030 }, { 1090, 3220 }, { 1060, 3210 }, { 1010, 3210 }, { 970, 3220 },  { -740, 3940 } };
    EXPECT_EQ(smoothed.poly, expected);

    auto original_expected = std::vector<cura::Point>{ { -137, 188 },  { 1910, 540 },  { 3820, 540 },  { 3850, 640 },  { 5040, 780 },  { 5660, 2800 }, { 5420, 2720 }, { 5500, 2850 },
                                                       { 5530, 2970 }, { 5290, 3450 }, { 1610, 4030 }, { 1090, 3220 }, { 1060, 3210 }, { 1010, 3210 }, { 970, 3220 },  { -740, 3940 } };
    EXPECT_EQ(poly.poly, original_expected);

    // test isWithinAllowedDeviations utility function
    auto FLUID_ANGLE = std::cos(10. * M_PI / 180.);
    auto MAX_RESOUTION = 250;
    cura::actions::smooth_fn smooth;

    {
        /*
         *
         *  A ------------- B
         *                  |
         *                  C --------------- D
         *
         */
        auto A = std::make_tuple(0, 0);
        auto B = std::make_tuple(0, 100);
        auto C = std::make_tuple(1, 100);
        auto D = std::make_tuple(1, 200);

        const auto is_within_allowed_deviation = smooth.isWithinAllowedDeviations(&A, &B, &C, &D, FLUID_ANGLE, MAX_RESOUTION, 100., 1., 100.);
        EXPECT_EQ(is_within_allowed_deviation, false);
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
        auto A = std::make_tuple(0, 0);
        auto B = std::make_tuple(100, 0);
        auto C = std::make_tuple(101, 1);
        auto D = std::make_tuple(101, 101);

        const auto is_within_allowed_deviation = smooth.isWithinAllowedDeviations(&A, &B, &C, &D, FLUID_ANGLE, MAX_RESOUTION, 100., std::sqrt(2), 100.);
        EXPECT_EQ(is_within_allowed_deviation, true);
    }

    {
        /*
         *
         *  A ----------- B - C -------------D
         *
         */
        auto A = std::make_tuple(0, 0);
        auto B = std::make_tuple(100, 0);
        auto C = std::make_tuple(101, 0);
        auto D = std::make_tuple(201, 0);

        const auto is_within_allowed_deviation = smooth.isWithinAllowedDeviations(&A, &B, &C, &D, FLUID_ANGLE, MAX_RESOUTION, 100., 1., 100.);
        EXPECT_EQ(is_within_allowed_deviation, true);
    }

    {
        /*
         *
         *  D ----------- C - B -------------A
         *
         */
        auto A = std::make_tuple(201, 0);
        auto B = std::make_tuple(101, 0);
        auto C = std::make_tuple(100, 0);
        auto D = std::make_tuple(0, 0);

        const auto is_within_allowed_deviation = smooth.isWithinAllowedDeviations(&A, &B, &C, &D, FLUID_ANGLE, MAX_RESOUTION, 100., 1., 100.);
        EXPECT_EQ(is_within_allowed_deviation, true);
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
        auto A = std::make_tuple(0, 0);
        auto B = std::make_tuple(100, 0);
        auto C = std::make_tuple(99, -1);
        auto D = std::make_tuple(199, 99);

        const auto is_within_allowed_deviation = smooth.isWithinAllowedDeviations(&A, &B, &C, &D, FLUID_ANGLE, MAX_RESOUTION, 100., std::sqrt(2.), std::sqrt(100. * 100. + 100. * 100.));
        EXPECT_EQ(is_within_allowed_deviation, true);
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
        auto A = std::make_tuple(0, 0);
        auto B = std::make_tuple(100, 0);
        auto C = std::make_tuple(101, 1);
        auto D = std::make_tuple(201, 101);

        const auto is_within_allowed_deviation = smooth.isWithinAllowedDeviations(&A, &B, &C, &D, FLUID_ANGLE, MAX_RESOUTION, 100., std::sqrt(2.), std::sqrt(100. * 100. + 100. * 100.));
        EXPECT_EQ(is_within_allowed_deviation, true);
    }

    {
        /*
         *
         *  D ----------- C - B
         *                    |
         *                    |
         *                    |
         *                    |
         *                    D
         *
         */
        auto A = std::make_tuple(0, 0);
        auto B = std::make_tuple(100, 0);
        auto C = std::make_tuple(101, 0);
        auto D = std::make_tuple(101, 100);

        const auto is_within_allowed_deviation = smooth.isWithinAllowedDeviations(&A, &B, &C, &D, FLUID_ANGLE, MAX_RESOUTION, 100., 1., 100.);
        EXPECT_EQ(is_within_allowed_deviation, true);
    }

}
} // namespace cura