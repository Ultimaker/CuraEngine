// Copyright (c) 2026 CuraEngine contributors
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/CrossHatchInfill.h"

#include <cmath>

#include <gtest/gtest.h>

#include "geometry/OpenLinesSet.h"
#include "geometry/Shape.h"
#include "settings/EnumSettings.h"
#include "settings/Settings.h"
#include "utils/AABB.h"
#include "utils/Coord_t.h"

namespace cura
{
namespace
{

class RawCrossHatchInfill : public CrossHatchInfill
{
public:
    using CrossHatchInfill::generateParallelLines;
};

Shape squareWithHole()
{
    Shape shape;
    shape.push_back(Polygon{ { { -20000, -20000 }, { 20000, -20000 }, { 20000, 20000 }, { -20000, 20000 } }, false });
    shape.push_back(Polygon{ { { -5000, -5000 }, { -5000, 5000 }, { 5000, 5000 }, { 5000, -5000 } }, false });
    return shape;
}

} // namespace

TEST(CrossHatchInfillTest, ParsesPatternSetting)
{
    Settings settings;
    settings.add("infill_pattern", "crosshatch");
    EXPECT_EQ(settings.get<EFillMethod>("infill_pattern"), EFillMethod::CROSSHATCH);
}

TEST(CrossHatchInfillTest, RejectsInvalidSpacing)
{
    const AABB bounds(squareWithHole());
    const RawCrossHatchInfill generator;
    EXPECT_TRUE(generator.generateParallelLines(0, bounds, 200, 400).empty());
    EXPECT_TRUE(generator.generateParallelLines(-100, bounds, 200, 400).empty());
    EXPECT_TRUE(generator.generateParallelLines(2000, bounds, 200, 0).empty());
}

TEST(CrossHatchInfillTest, StraightStacksAlternateByNinetyDegrees)
{
    const RawCrossHatchInfill generator;
    const AABB bounds(squareWithHole());
    constexpr coord_t line_distance = 2000;
    constexpr coord_t line_width = 400;
    constexpr double spacing = line_distance * 1.08;
    const double repeat_height = spacing * (1.0 - std::exp(-1.0));
    const double half_period = repeat_height + spacing * 0.4;

    for (int stack = 0; stack < 2; ++stack)
    {
        const coord_t z = std::llrint(repeat_height + stack * half_period);
        const OpenLinesSet lines = generator.generateParallelLines(line_distance, bounds, z, line_width);
        ASSERT_FALSE(lines.empty());
        for (const OpenPolyline& line : lines)
        {
            ASSERT_EQ(line.size(), 2);
            if (stack == 0)
            {
                EXPECT_EQ(line.front().Y, line.back().Y);
                EXPECT_NE(line.front().X, line.back().X);
            }
            else
            {
                EXPECT_EQ(line.front().X, line.back().X);
                EXPECT_NE(line.front().Y, line.back().Y);
            }
        }
    }
}

TEST(CrossHatchInfillTest, TransitionLayersContainCorrugatedLines)
{
    const OpenLinesSet lines = RawCrossHatchInfill().generateParallelLines(2000, AABB(squareWithHole()), 0, 400);
    ASSERT_FALSE(lines.empty());
    for (const OpenPolyline& line : lines)
    {
        ASSERT_GT(line.size(), 4);
        bool has_diagonal = false;
        for (size_t i = 1; i < line.size(); ++i)
        {
            const Point2LL delta = line[i] - line[i - 1];
            EXPECT_TRUE(delta.X != 0 || delta.Y != 0);
            has_diagonal |= delta.X != 0 && delta.Y != 0;
        }
        EXPECT_TRUE(has_diagonal);
    }
}

TEST(CrossHatchInfillTest, FitsRotatedConcaveAndHoledRegions)
{
    Shape outline = squareWithHole();
    // Cut a notch into the right edge as well as retaining the central hole.
    Shape notch;
    notch.push_back(Polygon{ { { 10000, -10000 }, { 25000, -10000 }, { 25000, 0 }, { 10000, 0 } }, false });
    outline = outline.difference(notch);

    for (const coord_t distance : { 800, 2000, 8000, 20000 })
    {
        for (const coord_t z : { 0, 200, 1000, 3000, 10000 })
        {
            for (const bool connect : { false, true })
            {
                SCOPED_TRACE(testing::Message() << "distance=" << distance << ", z=" << z << ", connect=" << connect);
                OpenLinesSet lines;
                Shape polygons;
                CrossHatchInfill().generateInfill(lines, polygons, connect, distance, outline, z, 400, AngleDegrees(33));
                ASSERT_FALSE(lines.empty() && polygons.empty());
                // A small tolerance allows for integer rounding during rotation and clipping.
                const Shape padded = outline.offset(5);
                EXPECT_NEAR(padded.intersection(lines, false).length(), lines.length(), 10);
                OpenLinesSet closed_lines;
                for (const Polygon& polygon : polygons)
                {
                    closed_lines.push_back(polygon.toPseudoOpenPolyline());
                }
                EXPECT_NEAR(padded.intersection(closed_lines, false).length(), closed_lines.length(), 10);
            }
        }
    }
}

TEST(CrossHatchInfillTest, PatternRemainsAlignedWhenAnIslandChangesBounds)
{
    const Shape outline = squareWithHole();
    const AABB bounds(outline);
    AABB larger_bounds = bounds;
    larger_bounds.min_ -= Point2LL(15000, 17000);
    larger_bounds.max_ += Point2LL(11000, 13000);
    const RawCrossHatchInfill generator;

    for (const coord_t z : { 0, 200, 1000, 3000 })
    {
        const OpenLinesSet first = outline.intersection(generator.generateParallelLines(2000, bounds, z, 400), false);
        const OpenLinesSet second = outline.intersection(generator.generateParallelLines(2000, larger_bounds, z, 400), false);
        ASSERT_FALSE(first.empty());
        ASSERT_EQ(first.size(), second.size());
        EXPECT_NEAR(first.length(), second.length(), 10);
        // Check geometry as well as length, independent of clipping's path order.
        for (const OpenPolyline& line : first)
        {
            for (const Point2LL& point : line)
            {
                bool found = false;
                for (const OpenPolyline& other : second)
                {
                    for (const Point2LL& candidate : other)
                    {
                        found |= point == candidate;
                    }
                }
                EXPECT_TRUE(found);
            }
        }
    }
}

TEST(CrossHatchInfillTest, AverageMaterialFractionMatchesRequestedDensity)
{
    const Shape outline(Polygon{ { { -100000, -100000 }, { 100000, -100000 }, { 100000, 100000 }, { -100000, 100000 } }, false });
    const AABB bounds(outline);
    const RawCrossHatchInfill generator;
    constexpr coord_t line_width = 400;
    constexpr coord_t line_distance = 2000;
    constexpr double spacing = line_distance * 1.08;
    const double period = 2 * spacing * (1.0 - std::exp(-1.0) + 0.4);
    constexpr int samples = 64;
    double total_length = 0;
    for (int sample = 0; sample < samples; ++sample)
    {
        const coord_t z = std::llrint(period * sample / samples);
        total_length += outline.intersection(generator.generateParallelLines(line_distance, bounds, z, line_width), false).length();
    }
    const double fraction = total_length * line_width / (samples * outline.area());
    EXPECT_NEAR(fraction, 0.2, 0.02);
}

} // namespace cura
