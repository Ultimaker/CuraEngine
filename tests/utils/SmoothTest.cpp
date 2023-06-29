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
    Polygon poly;
    poly.poly = std::vector<cura::Point>{ { -137, 188 },  { 1910, 540 },  { 3820, 540 },  { 3850, 640 },  { 5040, 780 },  { 5660, 2800 }, { 5420, 2720 }, { 5500, 2850 },
                                          { 5530, 2970 }, { 5290, 3450 }, { 1610, 4030 }, { 1090, 3220 }, { 1060, 3210 }, { 1010, 3210 }, { 970, 3220 },  { -740, 3940 } };
    auto smoother = cura::actions::smooth(750, 100, 5.0);
    Polygon smoothed;
    smoothed.poly = smoother(poly.poly);

    std::vector<cura::Point> expected{ { -137, 188 },  { 1910, 540 },  { 3720, 540 },  { 3949, 651 },  { 5040, 780 },  { 5631, 2705 }, { 5420, 2720 }, { 5500, 2850 },
                                       { 5486, 3059 }, { 5290, 3450 }, { 1610, 4030 }, { 1144, 3304 }, { 1060, 3210 }, { 1010, 3210 }, { 878, 3258 },  { -740, 3940 } };
    EXPECT_EQ(smoothed.poly, expected);

    auto original_expected = std::vector<cura::Point>{ { -137, 188 },  { 1910, 540 },  { 3820, 540 },  { 3850, 640 },  { 5040, 780 },  { 5660, 2800 }, { 5420, 2720 }, { 5500, 2850 },
                                                       { 5530, 2970 }, { 5290, 3450 }, { 1610, 4030 }, { 1090, 3220 }, { 1060, 3210 }, { 1010, 3210 }, { 970, 3220 },  { -740, 3940 } };
    EXPECT_EQ(poly.poly, original_expected);
}
} // namespace cura