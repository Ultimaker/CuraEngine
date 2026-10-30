// Copyright (c) 2026 CuraEngine contributors
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/CrossHatchInfill.h"

#include <algorithm>
#include <cmath>
#include <utility>

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

bool isVertical(const OpenLinesSet& lines)
{
    const Point2LL span = lines.front().back() - lines.front().front();
    return std::abs(span.Y) > std::abs(span.X);
}

bool hasDiagonals(const OpenLinesSet& lines)
{
    for (const OpenPolyline& line : lines)
    {
        for (size_t i = 1; i < line.size(); ++i)
        {
            const Point2LL delta = line[i] - line[i - 1];
            if (delta.X != 0 && delta.Y != 0)
            {
                return true;
            }
        }
    }
    return false;
}

// Find a geometry change without using the generator's phase/spacing formula.
// Resolve its height to a single micron so continuity can be tested directly.
template<typename Classify>
std::pair<coord_t, coord_t> findChange(const coord_t distance, const coord_t line_width, const coord_t start_z, Classify classify)
{
    const RawCrossHatchInfill generator;
    const AABB bounds(Point2LL(-8 * distance, -8 * distance), Point2LL(8 * distance, 8 * distance));
    const bool initial = classify(generator.generateParallelLines(distance, bounds, start_z, line_width));
    const coord_t step = std::max(coord_t{ 1 }, distance / 20);
    coord_t lower = start_z;
    for (int i = 1; i <= 128; ++i)
    {
        coord_t upper = start_z + i * step;
        if (classify(generator.generateParallelLines(distance, bounds, upper, line_width)) == initial)
        {
            lower = upper;
            continue;
        }
        while (upper - lower > 1)
        {
            const coord_t middle = (lower + upper) / 2;
            if (classify(generator.generateParallelLines(distance, bounds, middle, line_width)) == initial)
            {
                lower = middle;
            }
            else
            {
                upper = middle;
            }
        }
        return { lower, upper };
    }
    ADD_FAILURE() << "Could not find a change in the Cross Hatch geometry";
    return { 0, 0 };
}

Shape interiorSquare(const coord_t distance)
{
    const coord_t extent = 4 * distance;
    return Shape(Polygon{ { { -extent, -extent }, { extent, -extent }, { extent, extent }, { -extent, extent } }, false });
}

OpenLinesSet diagonals(const OpenLinesSet& lines)
{
    OpenLinesSet result;
    for (const OpenPolyline& line : lines)
    {
        for (size_t i = 1; i < line.size(); ++i)
        {
            const Point2LL delta = line[i] - line[i - 1];
            if (delta.X != 0 && delta.Y != 0)
            {
                result.addSegment(line[i - 1], line[i]);
            }
        }
    }
    return result;
}

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
    bool saw_horizontal = false;
    bool saw_vertical = false;
    for (coord_t z = 0; z < line_distance * 6; z += 100)
    {
        const OpenLinesSet lines = generator.generateParallelLines(line_distance, bounds, z, line_width);
        ASSERT_FALSE(lines.empty());
        if (hasDiagonals(lines))
        {
            continue;
        }
        const bool vertical = isVertical(lines);
        saw_vertical |= vertical;
        saw_horizontal |= ! vertical;
        for (const OpenPolyline& line : lines)
        {
            ASSERT_EQ(line.size(), 2);
            if (! vertical)
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
    EXPECT_TRUE(saw_horizontal);
    EXPECT_TRUE(saw_vertical);
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

TEST(CrossHatchInfillTest, AppendingARotatedIslandPreservesExistingPaths)
{
    const OpenPolyline previous_line{ { -90000, -90000 }, { -80000, -90000 } };
    const Polygon previous_polygon{ { { -90000, -80000 }, { -80000, -80000 }, { -80000, -70000 } }, false };
    OpenLinesSet lines(previous_line);
    Shape polygons(previous_polygon);
    CrossHatchInfill().generateInfill(lines, polygons, true, 2000, squareWithHole(), 1000, 400, AngleDegrees(33));
    ASSERT_GE(lines.size(), 1);
    ASSERT_GE(polygons.size(), 1);
    ASSERT_EQ(lines.front().size(), previous_line.size());
    ASSERT_EQ(polygons.front().size(), previous_polygon.size());
    for (size_t i = 0; i < previous_line.size(); ++i)
    {
        EXPECT_EQ(lines.front()[i], previous_line[i]);
    }
    for (size_t i = 0; i < previous_polygon.size(); ++i)
    {
        EXPECT_EQ(polygons.front()[i], previous_polygon[i]);
    }
}

TEST(CrossHatchInfillTest, AverageMaterialFractionMatchesRequestedDensity)
{
    const RawCrossHatchInfill generator;
    constexpr coord_t line_width = 400;
    for (const double density : { 0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 30.0, 50.0, 80.0, 100.0 })
    {
        SCOPED_TRACE(testing::Message() << "density=" << density);
        const coord_t distance = std::llrint(line_width * 100 / density);
        const coord_t extent = distance * 20;
        const Shape outline(Polygon{ { { -extent, -extent }, { extent, -extent }, { extent, extent }, { -extent, extent } }, false });
        const AABB bounds(outline);
        const auto first = findChange(distance, line_width, 0, isVertical);
        const auto second = findChange(distance, line_width, first.second + 1, isVertical);
        const coord_t period = 2 * (second.second - first.second);
        constexpr int samples = 128;
        double total_length = 0;
        for (int sample = 0; sample < samples; ++sample)
        {
            const coord_t z = period * sample / samples;
            total_length += outline.intersection(generator.generateParallelLines(distance, bounds, z, line_width), false).length();
        }
        const double fraction = total_length * line_width / (samples * outline.area());
        const double expected = density / 100.0;
        EXPECT_NEAR(fraction, expected, expected * 0.04);
    }
}

TEST(CrossHatchInfillTest, BothDirectionSwitchesShareDiagonalsAtEveryDensity)
{
    const RawCrossHatchInfill generator;
    std::vector<double> densities{ 0.1, 0.5 };
    for (int density = 1; density <= 100; ++density)
    {
        densities.push_back(density);
    }
    for (const double density : densities)
    {
        for (const coord_t line_width : { 200, 400, 800 })
        {
            SCOPED_TRACE(testing::Message() << "density=" << density << ", line_width=" << line_width);
            const coord_t distance = std::llrint(line_width * 100 / density);
            const AABB bounds(Point2LL(-8 * distance, -8 * distance), Point2LL(8 * distance, 8 * distance));
            const Shape interior = interiorSquare(distance);
            coord_t start_z = 0;
            for (int direction = 0; direction < 2; ++direction)
            {
                const auto [before_z, after_z] = findChange(distance, line_width, start_z, isVertical);
                ASSERT_LT(before_z, after_z);
                const OpenLinesSet before = diagonals(generator.generateParallelLines(distance, bounds, before_z, line_width));
                const OpenLinesSet after = interior.intersection(diagonals(generator.generateParallelLines(distance, bounds, after_z, line_width)), false);
                ASSERT_FALSE(before.empty());
                ASSERT_FALSE(after.empty());
                // At a direction switch, all diagonal paths must continue on
                // the same lattice, allowing only integer-coordinate rounding.
                const double supported = before.offset(5).intersection(after, false).length();
                EXPECT_GE(supported / after.length(), 0.995);
                start_z = after_z + 1;
            }
        }
    }
}

TEST(CrossHatchInfillTest, TransitionsJoinStraightStacksWithoutAPositionJump)
{
    const RawCrossHatchInfill generator;
    for (const coord_t distance : { 400, 800, 2000, 8000, 20000, 40000, 80000 })
    {
        SCOPED_TRACE(testing::Message() << "distance=" << distance);
        const AABB bounds(Point2LL(-8 * distance, -8 * distance), Point2LL(8 * distance, 8 * distance));
        const Shape interior = interiorSquare(distance);
        coord_t start_z = 0;
        for (int boundary = 0; boundary < 4; ++boundary)
        {
            const auto [before_z, after_z] = findChange(distance, 400, start_z, hasDiagonals);
            ASSERT_LT(before_z, after_z);
            const OpenLinesSet before = generator.generateParallelLines(distance, bounds, before_z, 400);
            const OpenLinesSet after = interior.intersection(generator.generateParallelLines(distance, bounds, after_z, 400), false);
            ASSERT_FALSE(after.empty());
            EXPECT_GE(static_cast<double>(before.offset(5).intersection(after, false).length()) / after.length(), 0.995);
            start_z = after_z + 1;
        }
    }
}

TEST(CrossHatchInfillTest, SampledTransitionLayersBondAtMultipleDensitiesAndLayerHeights)
{
    const RawCrossHatchInfill generator;
    for (const int density : { 1, 2, 5, 10, 20, 30, 50, 80, 100 })
    {
        constexpr coord_t line_width = 400;
        const coord_t distance = line_width * 100 / density;
        const AABB bounds(Point2LL(-8 * distance, -8 * distance), Point2LL(8 * distance, 8 * distance));
        const Shape interior = interiorSquare(distance);
        const auto [before_z, switch_z] = findChange(distance, line_width, 0, isVertical);
        ASSERT_LT(before_z, switch_z);
        for (const coord_t layer_height : { 80, 120, 200, 280, 400 })
        {
            SCOPED_TRACE(testing::Message() << "density=" << density << ", layer_height=" << layer_height);
            const coord_t lower_z = (switch_z - 1) / layer_height * layer_height;
            const OpenLinesSet before = generator.generateParallelLines(distance, bounds, lower_z, line_width);
            const OpenLinesSet raw_after = generator.generateParallelLines(distance, bounds, lower_z + layer_height, line_width);
            const OpenLinesSet after = interior.intersection(hasDiagonals(raw_after) ? diagonals(raw_after) : raw_after, false);
            ASSERT_FALSE(after.empty());
            // A full line-width distance permits contact between the extrusion
            // ribbons on consecutive layers, even with different path angles.
            EXPECT_GE(static_cast<double>(before.offset(line_width).intersection(after, false).length()) / after.length(), 0.9);
        }
    }
}

} // namespace cura
