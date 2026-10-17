// Copyright (c) 2025 UltiMaker and contributors
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/BoneInfill.h" // The unit under test.

#include <algorithm>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "geometry/OpenLinesSet.h"
#include "geometry/OpenPolyline.h"
#include "geometry/Polygon.h"
#include "geometry/Shape.h"
#include "settings/types/Angle.h"
#include "utils/Coord_t.h"

// NOLINTBEGIN(*-magic-numbers)
namespace cura
{

class BoneInfillTest : public testing::Test
{
public:
    static constexpr coord_t LINE_WIDTH = 400;
    static constexpr coord_t SIZE = MM2INT(40);

    Shape square;

    BoneInfillTest()
    {
        square.emplace_back();
        square.back().emplace_back(0, 0);
        square.back().emplace_back(SIZE, 0);
        square.back().emplace_back(SIZE, SIZE);
        square.back().emplace_back(0, SIZE);
    }

    struct Result
    {
        OpenLinesSet lines;
        Shape polygons;
    };

    Result generate(const BoneParameters& parameters, const coord_t line_distance, const coord_t z, const Shape* outline = nullptr) const
    {
        Result result;
        BoneInfill(parameters).generateInfill(result.lines, result.polygons, line_distance, outline ? *outline : square, z, LINE_WIDTH, AngleDegrees(0.0));
        return result;
    }

    static BoneParameters withoutDenseZone(BoneParameters parameters = {})
    {
        parameters.cortical_width = 0;
        return parameters;
    }

    static double totalLength(const OpenLinesSet& lines)
    {
        double length = 0.0;
        for (const OpenPolyline& line : lines)
        {
            for (size_t i = 1; i < line.size(); ++i)
            {
                length += std::hypot(static_cast<double>(line[i].X - line[i - 1].X), static_cast<double>(line[i].Y - line[i - 1].Y));
            }
        }
        return length;
    }

    static std::vector<Point2LL> samplePoints(const OpenLinesSet& lines, const coord_t spacing = 200)
    {
        std::vector<Point2LL> points;
        for (const OpenPolyline& line : lines)
        {
            for (size_t i = 1; i < line.size(); ++i)
            {
                const double length = std::hypot(static_cast<double>(line[i].X - line[i - 1].X), static_cast<double>(line[i].Y - line[i - 1].Y));
                const int steps = std::max(1, static_cast<int>(length / spacing));
                for (int s = 0; s < steps; ++s)
                {
                    const double t = static_cast<double>(s) / steps;
                    points.emplace_back(line[i - 1].X + std::llround((line[i].X - line[i - 1].X) * t), line[i - 1].Y + std::llround((line[i].Y - line[i - 1].Y) * t));
                }
            }
        }
        return points;
    }

    /*! The fraction of the points of one layer that have a point of the other layer within the given distance. */
    static double fractionNear(const std::vector<Point2LL>& points, const std::vector<Point2LL>& others, const coord_t distance)
    {
        size_t near = 0;
        const double distance_squared = static_cast<double>(distance) * static_cast<double>(distance);
        for (const Point2LL& p : points)
        {
            for (const Point2LL& q : others)
            {
                const double dx = static_cast<double>(p.X - q.X);
                const double dy = static_cast<double>(p.Y - q.Y);
                if (dx * dx + dy * dy <= distance_squared)
                {
                    ++near;
                    break;
                }
            }
        }
        return points.empty() ? 0.0 : static_cast<double>(near) / static_cast<double>(points.size());
    }

    static std::vector<Point2LL> thinned(const std::vector<Point2LL>& points, const size_t every)
    {
        std::vector<Point2LL> result;
        for (size_t i = 0; i < points.size(); i += every)
        {
            result.push_back(points[i]);
        }
        return result;
    }
};

TEST_F(BoneInfillTest, NothingWithoutLineDistanceOrOutline)
{
    EXPECT_TRUE(generate({}, 0, 1000).lines.empty()) << "A line distance of zero cannot make a pattern.";
    const Shape empty_outline;
    EXPECT_TRUE(generate({}, MM2INT(3), 1000, &empty_outline).lines.empty()) << "There is nothing to fill in an empty outline.";
}

TEST_F(BoneInfillTest, ProducesLines)
{
    const Result result = generate(withoutDenseZone(), MM2INT(3), 5000);
    EXPECT_FALSE(result.lines.empty());
    EXPECT_TRUE(result.polygons.empty()) << "Closed loops are handed over as lines, so that they are not joined by straight bridges.";
}

TEST_F(BoneInfillTest, SameSeedGivesSamePattern)
{
    BoneParameters parameters = withoutDenseZone();
    parameters.seed = 7;
    const Result first = generate(parameters, MM2INT(3), 5000);
    const Result second = generate(parameters, MM2INT(3), 5000);

    ASSERT_EQ(first.lines.size(), second.lines.size());
    for (size_t i = 0; i < first.lines.size(); ++i)
    {
        ASSERT_EQ(first.lines[i].size(), second.lines[i].size());
        for (size_t j = 0; j < first.lines[i].size(); ++j)
        {
            EXPECT_EQ(first.lines[i][j], second.lines[i][j]);
        }
    }
}

TEST_F(BoneInfillTest, DifferentSeedsGiveDifferentPatterns)
{
    BoneParameters first_parameters = withoutDenseZone();
    first_parameters.seed = 7;
    BoneParameters second_parameters = first_parameters;
    second_parameters.seed = 8;
    const Result first = generate(first_parameters, MM2INT(3), 5000);
    const Result second = generate(second_parameters, MM2INT(3), 5000);

    // Little of the first pattern can be found on the second.
    EXPECT_LT(fractionNear(thinned(samplePoints(first.lines), 5), samplePoints(second.lines), 100), 0.3);
}

TEST_F(BoneInfillTest, StaysInsideTheOutline)
{
    const Shape slightly_larger = square.offset(10);
    const Result result = generate({}, MM2INT(3), 5000);
    ASSERT_FALSE(result.lines.empty());
    for (const Point2LL& point : samplePoints(result.lines))
    {
        EXPECT_TRUE(slightly_larger.inside(point)) << "(" << point.X << ", " << point.Y << ") is outside of the outline.";
    }
}

TEST_F(BoneInfillTest, AmountOfInfillFollowsLineDistance)
{
    for (const coord_t line_distance : { MM2INT(1.5), MM2INT(3), MM2INT(5) })
    {
        double total = 0.0;
        constexpr int layers = 10;
        for (int layer = 0; layer < layers; ++layer)
        {
            total += totalLength(generate(withoutDenseZone(), line_distance, MM2INT(2) + layer * 200).lines);
        }
        const double expected = INT2MM(SIZE) * INT2MM(SIZE) / INT2MM(line_distance);
        EXPECT_NEAR(INT2MM(total / layers) / expected, 1.0, 0.15) << "With a line distance of " << INT2MM(line_distance) << "mm.";
    }
}

TEST_F(BoneInfillTest, LayersAreConnected)
{
    // Every layer must be supported by the one below it: nearly all of the infill has infill within a line width below it.
    const std::vector<Point2LL> lower = samplePoints(generate({}, MM2INT(3), MM2INT(2)).lines);
    const std::vector<Point2LL> upper = samplePoints(generate({}, MM2INT(3), MM2INT(2) + 200).lines);
    EXPECT_GT(fractionNear(thinned(upper, 3), lower, LINE_WIDTH), 0.9);
}

TEST_F(BoneInfillTest, AlignmentMakesStrutsUpright)
{
    // Struts that stand upright move very little from one layer to the next.
    BoneParameters isotropic = withoutDenseZone();
    isotropic.alignment = 0.0;
    BoneParameters aligned = withoutDenseZone();
    aligned.alignment = 0.95;

    const auto stays_in_place = [this](const BoneParameters& parameters)
    {
        const std::vector<Point2LL> lower = samplePoints(generate(parameters, MM2INT(3), MM2INT(2)).lines);
        const std::vector<Point2LL> upper = samplePoints(generate(parameters, MM2INT(3), MM2INT(2) + 200).lines);
        return fractionNear(thinned(upper, 3), lower, 150);
    };

    EXPECT_GT(stays_in_place(aligned), stays_in_place(isotropic) + 0.15);
}

TEST_F(BoneInfillTest, LowerConnectivityGivesMoreClosedLoops)
{
    const auto closed_fraction = [this](const double connectivity)
    {
        BoneParameters parameters = withoutDenseZone();
        parameters.connectivity = connectivity;
        const Result result = generate(parameters, MM2INT(3), 5000);
        size_t closed = 0;
        for (const OpenPolyline& line : result.lines)
        {
            closed += (line.size() > 2 && line.front() == line.back()) ? 1 : 0;
        }
        return static_cast<double>(closed) / static_cast<double>(std::max<size_t>(1, result.lines.size()));
    };

    EXPECT_GT(closed_fraction(0.3), closed_fraction(1.0)) << "A less connected structure breaks up into separate cells.";
}

TEST_F(BoneInfillTest, DenseZoneAddsInfillNextToTheWallsOnly)
{
    BoneParameters dense;
    dense.cortical_width = MM2INT(4);
    dense.cortical_lines = 1;

    const auto near_wall_and_middle = [this](const BoneParameters& parameters)
    {
        double near_wall = 0.0;
        double middle = 0.0;
        for (const Point2LL& p : samplePoints(generate(parameters, MM2INT(3), 5000).lines))
        {
            const coord_t distance_to_wall = std::min<coord_t>({ p.X, SIZE - p.X, p.Y, SIZE - p.Y });
            near_wall += distance_to_wall < MM2INT(4) ? 1.0 : 0.0;
            middle += distance_to_wall > MM2INT(6) ? 1.0 : 0.0;
        }
        return std::make_pair(near_wall, middle);
    };

    const auto [dense_near, dense_middle] = near_wall_and_middle(dense);
    const auto [plain_near, plain_middle] = near_wall_and_middle(withoutDenseZone());
    EXPECT_GT(dense_near / plain_near, 1.5);
    EXPECT_NEAR(dense_middle / plain_middle, 1.0, 0.1);
}

} // namespace cura
// NOLINTEND(*-magic-numbers)
