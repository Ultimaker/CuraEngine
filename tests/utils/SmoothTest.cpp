// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <gtest/gtest.h>

#include <range/v3/all.hpp>
#include <set>

#include "utils/IntPoint.h"
#include "utils/actions/smooth.h"
#include "utils/polygon.h"

namespace cura
{

TEST(SmoothTest, TestSmooth)
{
    Polygon poly;
    poly.poly = std::vector<cura::Point>{ cura::Point{ 0, 0 }, { 1000, 1000 }, { 1020, 1010 }, { 3000, 0 }, { 10, 0 }, { 10, 10 }, { 5, 5 } };
    auto smoother = cura::actions::smooth(100, 100, 10.0);
    auto smoothed = smoother(poly.poly);
    std::vector<cura::Point> expected{ cura::Point{ 0, 0 }, { 1000, 1000 }, { 1020, 1010 }, { 3000, 0 }, { 10, 0 } };
    EXPECT_EQ(smoothed, expected);
}
} // namespace cura