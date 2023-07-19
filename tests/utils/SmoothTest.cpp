// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/actions/smooth.h"

#include "utils/IntPoint.h"
#include "utils/polygon.h"

#include <range/v3/all.hpp>

#include <gtest/gtest.h>
#include <set>

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

    auto original_expected
        = std::vector<cura::Point>{ { -137, 188 },  { 1910, 540 },  { 3820, 540 },  { 3850, 640 },  { 5040, 780 },  { 5660, 2800 }, { 5420, 2720 }, { 5500, 2850 },
                                    { 5530, 2970 }, { 5290, 3450 }, { 1610, 4030 }, { 1090, 3220 }, { 1060, 3210 }, { 1010, 3210 }, { 970, 3220 },  { -740, 3940 } };
    EXPECT_EQ(poly.poly, original_expected);
}
} // namespace cura