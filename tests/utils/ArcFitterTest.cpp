// Copyright (c) 2025 UltiMaker and contributors
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/ArcFitter.h" // The unit under test.

#include <cmath>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

#include "geometry/Point2LL.h"
#include "utils/Coord_t.h"

// NOLINTBEGIN(*-magic-numbers)
namespace cura
{

class ArcFitterTest : public testing::Test
{
public:
    static constexpr coord_t TOLERANCE = 50; // 0.05mm
    static constexpr coord_t MAX_RADIUS = 1'000'000; // 1000mm
    static constexpr coord_t MIN_RADIUS = 300; // 0.3mm

    /*! Points on a circle (microns) from one angle to another, in steps. The direction follows the sign of the step. */
    static std::vector<Point2LL>
        circle(const double center_x_mm, const double center_y_mm, const double radius_mm, const double from_degrees, const double to_degrees, const double step_degrees)
    {
        std::vector<Point2LL> points;
        const double direction = (to_degrees >= from_degrees) ? 1.0 : -1.0;
        for (double angle = from_degrees; (to_degrees - angle) * direction > -1e-9; angle += direction * std::abs(step_degrees))
        {
            const double radians = angle * std::numbers::pi / 180.0;
            points.emplace_back(MM2INT(center_x_mm + radius_mm * std::cos(radians)), MM2INT(center_y_mm + radius_mm * std::sin(radians)));
        }
        return points;
    }

    static std::vector<ArcFitter::Segment> fit(const std::vector<Point2LL>& points, const coord_t max_radius = MAX_RADIUS, const coord_t min_radius = MIN_RADIUS)
    {
        return ArcFitter::fit(points, TOLERANCE, max_radius, min_radius);
    }

    /*! The segments have to join up and together cover all the points, from the first to the last. */
    static void expectCoversAllPoints(const std::vector<ArcFitter::Segment>& segments, const size_t point_count)
    {
        ASSERT_FALSE(segments.empty());
        EXPECT_EQ(segments.front().first_index, 0U);
        EXPECT_EQ(segments.back().last_index, point_count - 1);
        for (size_t i = 1; i < segments.size(); ++i)
        {
            EXPECT_EQ(segments[i].first_index, segments[i - 1].last_index) << "Segment " << i << " does not start where the previous one ended.";
        }
        for (const ArcFitter::Segment& segment : segments)
        {
            EXPECT_LT(segment.first_index, segment.last_index) << "Segments are never empty.";
        }
    }
};

TEST_F(ArcFitterTest, TooFewPoints)
{
    EXPECT_TRUE(fit({}).empty());
    EXPECT_TRUE(fit({ Point2LL(0, 0) }).empty());
}

TEST_F(ArcFitterTest, TwoPointsAreOneLine)
{
    const auto segments = fit({ Point2LL(0, 0), Point2LL(1000, 0) });
    ASSERT_EQ(segments.size(), 1U);
    EXPECT_FALSE(segments[0].is_arc);
}

TEST_F(ArcFitterTest, CounterClockwiseQuarterCircleIsOneArc)
{
    const auto points = circle(50, 50, 10, 0, 90, 5);
    const auto segments = fit(points);

    ASSERT_EQ(segments.size(), 1U);
    const ArcFitter::Segment& arc = segments[0];
    EXPECT_TRUE(arc.is_arc);
    EXPECT_FALSE(arc.clockwise);
    EXPECT_EQ(arc.first_index, 0U);
    EXPECT_EQ(arc.last_index, points.size() - 1);
    EXPECT_NEAR(arc.center_x, MM2INT(50), 5.0);
    EXPECT_NEAR(arc.center_y, MM2INT(50), 5.0);
    EXPECT_NEAR(arc.radius, MM2INT(10), 5.0);
    EXPECT_NEAR(arc.sweep, std::numbers::pi / 2.0, 0.001);
    EXPECT_NEAR(arc.arcLength(), MM2INT(10) * std::numbers::pi / 2.0, 20.0);
}

TEST_F(ArcFitterTest, ClockwiseArcIsRecognizedAsClockwise)
{
    const auto points = circle(0, 0, 10, 90, 0, 5);
    const auto segments = fit(points);

    ASSERT_EQ(segments.size(), 1U);
    EXPECT_TRUE(segments[0].is_arc);
    EXPECT_TRUE(segments[0].clockwise);
    EXPECT_NEAR(segments[0].sweep, std::numbers::pi / 2.0, 0.001);
}

TEST_F(ArcFitterTest, StraightLineIsNotAnArc)
{
    std::vector<Point2LL> points;
    for (int i = 0; i <= 20; ++i)
    {
        points.emplace_back(MM2INT(i * 0.5), MM2INT(i * 0.25));
    }
    const auto segments = fit(points);

    ASSERT_EQ(segments.size(), 1U);
    EXPECT_FALSE(segments[0].is_arc) << "Collinear points are a straight line, not a circle of infinite radius.";
    expectCoversAllPoints(segments, points.size());
}

TEST_F(ArcFitterTest, CornerIsNotRoundedOff)
{
    // Three points always have a circle through them. If that were enough, every corner would become an arc.
    const std::vector<Point2LL> right_angle = { Point2LL(0, 0), Point2LL(MM2INT(10), 0), Point2LL(MM2INT(10), MM2INT(10)) };
    for (const ArcFitter::Segment& segment : fit(right_angle))
    {
        EXPECT_FALSE(segment.is_arc) << "A right angle with long legs must stay a corner.";
    }

    const std::vector<Point2LL> obtuse = { Point2LL(0, 0), Point2LL(MM2INT(5), 0), Point2LL(MM2INT(5) + MM2INT(5) * 0.7, MM2INT(5) * 0.7) };
    for (const ArcFitter::Segment& segment : fit(obtuse))
    {
        EXPECT_FALSE(segment.is_arc) << "A corner of 135 degrees with long legs must stay a corner.";
    }
}

TEST_F(ArcFitterTest, SmallRoundedCornerBecomesAnArc)
{
    // Points close together on a tight bend do follow the circle, which is what rounded wall corners look like.
    const auto points = circle(0, 0, 1, 180, 270, 10);
    const auto segments = fit(points);

    ASSERT_EQ(segments.size(), 1U);
    EXPECT_TRUE(segments[0].is_arc);
    EXPECT_NEAR(segments[0].radius, MM2INT(1), 5.0);
}

TEST_F(ArcFitterTest, ArcFollowedByStraightLine)
{
    auto points = circle(0, 0, 10, 0, 90, 5); // ends at (0, 10), heading in the -x direction
    for (int i = 1; i <= 10; ++i)
    {
        points.emplace_back(MM2INT(-i * 1.0), MM2INT(10));
    }
    const auto segments = fit(points);

    expectCoversAllPoints(segments, points.size());
    ASSERT_GE(segments.size(), 2U);
    EXPECT_TRUE(segments.front().is_arc);
    EXPECT_FALSE(segments.back().is_arc);
    EXPECT_EQ(segments.back().last_index, points.size() - 1);
}

TEST_F(ArcFitterTest, SmallNoiseStillFits)
{
    // Points of real toolpaths lie on the curve to within rounding errors. The circle is determined by the first, middle and
    // last point, so noise on those moves the whole circle; a tenth of the tolerance is comfortably within what is accepted.
    auto points = circle(0, 0, 10, 0, 90, 3);
    for (size_t i = 1; i + 1 < points.size(); ++i)
    {
        const double offset = ((i % 2) ? 1 : -1) * TOLERANCE / 10.0;
        const double angle = std::atan2(static_cast<double>(points[i].Y), static_cast<double>(points[i].X));
        points[i].X += std::llround(offset * std::cos(angle));
        points[i].Y += std::llround(offset * std::sin(angle));
    }
    const auto segments = fit(points);

    ASSERT_EQ(segments.size(), 1U);
    EXPECT_TRUE(segments[0].is_arc);
}

TEST_F(ArcFitterTest, LargeNoiseDoesNotLoseAnyPoint)
{
    // With noise of a quarter of the tolerance on every point the arc may be cut into pieces, but nothing may get lost, and
    // the pieces that are arcs must still follow the points.
    auto points = circle(0, 0, 10, 0, 90, 3);
    for (size_t i = 1; i + 1 < points.size(); ++i)
    {
        const double offset = ((i % 2) ? 1 : -1) * TOLERANCE / 4.0;
        const double angle = std::atan2(static_cast<double>(points[i].Y), static_cast<double>(points[i].X));
        points[i].X += std::llround(offset * std::cos(angle));
        points[i].Y += std::llround(offset * std::sin(angle));
    }
    const auto segments = fit(points);

    expectCoversAllPoints(segments, points.size());
    for (const ArcFitter::Segment& segment : segments)
    {
        if (! segment.is_arc)
        {
            continue;
        }
        for (size_t i = segment.first_index; i <= segment.last_index; ++i)
        {
            const double distance = std::hypot(static_cast<double>(points[i].X) - segment.center_x, static_cast<double>(points[i].Y) - segment.center_y);
            EXPECT_NEAR(distance, segment.radius, TOLERANCE) << "Point " << i;
        }
    }
}

TEST_F(ArcFitterTest, PointFarOffTheCircleEndsTheArc)
{
    auto points = circle(0, 0, 10, 0, 90, 3);
    const size_t outlier = points.size() / 2;
    points[outlier].X += 4 * TOLERANCE; // well beyond the tolerance
    const auto segments = fit(points);

    expectCoversAllPoints(segments, points.size());
    for (const ArcFitter::Segment& segment : segments)
    {
        if (segment.is_arc)
        {
            const bool contains_outlier = segment.first_index <= outlier && outlier <= segment.last_index;
            EXPECT_FALSE(contains_outlier && segment.first_index != outlier && segment.last_index != outlier) << "An arc must not run through a point that is far off the circle.";
        }
    }
}

TEST_F(ArcFitterTest, ArcLargerThanMaximumRadiusStaysStraight)
{
    const auto points = circle(0, 0, 500, 0, 5, 0.1); // radius 500mm, segments of about 0.9mm
    EXPECT_TRUE(fit(points)[0].is_arc) << "Sanity check: this is an arc with the default limits.";

    for (const ArcFitter::Segment& segment : fit(points, MM2INT(100)))
    {
        EXPECT_FALSE(segment.is_arc);
    }
}

TEST_F(ArcFitterTest, ArcSmallerThanMinimumRadiusStaysStraight)
{
    const auto points = circle(0, 0, 0.2, 0, 180, 20); // radius 0.2mm, below the 0.3mm limit
    for (const ArcFitter::Segment& segment : fit(points))
    {
        EXPECT_FALSE(segment.is_arc);
    }
}

TEST_F(ArcFitterTest, FullCircleIsNeverOneArc)
{
    // Start and end of a full circle coincide, which G2/G3 would read as a different move; it has to be split.
    const auto points = circle(0, 0, 10, 0, 360, 5);
    const auto segments = fit(points);

    expectCoversAllPoints(segments, points.size());
    EXPECT_GE(segments.size(), 2U);
    for (const ArcFitter::Segment& segment : segments)
    {
        if (segment.is_arc)
        {
            EXPECT_LT(segment.sweep, 2.0 * std::numbers::pi * 0.98);
        }
    }
}

TEST_F(ArcFitterTest, ReversingDirectionEndsTheArc)
{
    auto points = circle(0, 0, 10, 0, 60, 5);
    auto back = circle(0, 0, 10, 55, 20, 5); // goes back over the same circle
    points.insert(points.end(), back.begin(), back.end());
    const auto segments = fit(points);

    expectCoversAllPoints(segments, points.size());
    for (const ArcFitter::Segment& segment : segments)
    {
        if (segment.is_arc)
        {
            // Within one arc the points all turn the same way, so the turning point (index 12) can only be at its end or start.
            EXPECT_FALSE(segment.first_index < 12 && segment.last_index > 12) << "An arc must not run through the point where the direction reverses.";
        }
    }
}

TEST_F(ArcFitterTest, EveryPointOfAnArcIsWithinTolerance)
{
    const auto points = circle(20, 30, 7.5, 10, 200, 4);
    const auto segments = fit(points);

    expectCoversAllPoints(segments, points.size());
    for (const ArcFitter::Segment& segment : segments)
    {
        if (! segment.is_arc)
        {
            continue;
        }
        for (size_t i = segment.first_index; i <= segment.last_index; ++i)
        {
            const double distance = std::hypot(static_cast<double>(points[i].X) - segment.center_x, static_cast<double>(points[i].Y) - segment.center_y);
            EXPECT_NEAR(distance, segment.radius, TOLERANCE) << "Point " << i;
        }
    }
}

TEST_F(ArcFitterTest, SegmentsAlwaysCoverAllPoints)
{
    // A winding path made of curves and straight stretches, with a fixed pseudo-random wobble.
    std::vector<Point2LL> points;
    double angle = 0.0;
    double x = 0.0;
    double y = 0.0;
    unsigned state = 12345;
    for (int i = 0; i < 400; ++i)
    {
        state = state * 1103515245U + 12345U;
        if ((state >> 16) % 40 == 0)
        {
            angle += ((state >> 8) % 2 ? 1 : -1) * 1.2; // an occasional sharp turn
        }
        else if (i % 100 < 50)
        {
            angle += 0.05; // a gentle curve
        }
        x += 0.4 * std::cos(angle);
        y += 0.4 * std::sin(angle);
        points.emplace_back(MM2INT(x), MM2INT(y));
    }
    expectCoversAllPoints(fit(points), points.size());
}

} // namespace cura
// NOLINTEND(*-magic-numbers)
