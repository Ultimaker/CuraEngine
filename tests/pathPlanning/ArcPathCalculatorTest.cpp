// Copyright (c) 2023 BigRep GmbH
#include "pathPlanning/ArcPathCalculator.h"

#include <gtest/gtest.h>

namespace cura
{
class ArcPathCalculation_GeneralCreationTest : public testing::Test
{
public:
    const Point2LL previous_position = { 0, 0 };
    const Point2LL current_position = { 80'000, 80'000 };
    const Point2LL target_position = { 10'000, 10'000 };
    const coord_t hop_height = 1'000;
    const coord_t radius = 10'000;
    const Velocity xy_speed = Velocity{ 1'000 };
    const Velocity z_speed = Velocity{ 500 };
    const coord_t step_size = 1;
    const coord_t n_discretization_steps = 10;
    const coord_t error_margin = 1;
};

TEST_F(ArcPathCalculation_GeneralCreationTest, CenterCorrectDistance)
{
    const ArcPath arc = ArcPath::calculate(previous_position, current_position, target_position, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_LT(vSize(current_position - arc.end_) - 2 * radius, error_margin);
}


TEST_F(ArcPathCalculation_GeneralCreationTest, CorrectTangentPoint_PositiveValues)
{
    const auto current_position = Point2LL{ 20'000, 10'000 };
    const auto target_position = Point2LL{ current_position.X + radius, current_position.Y + 2 * radius };
    const auto tangent_point = Point2LL{ current_position.X + radius, current_position.Y + radius };
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, current_position, target_position, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_LE(vSize(arc.end_ - tangent_point), error_margin);
    EXPECT_FALSE(arc.is_clockwise_);
}

TEST_F(ArcPathCalculation_GeneralCreationTest, CorrectTangentPoint_FromPositiveToNegativeValues)
{
    const auto current_position = Point2LL{ 20, 10 };
    const auto target_position = Point2LL{ current_position.X + radius, current_position.Y - 2 * radius };
    const auto tangent_point = Point2LL{ current_position.X + radius, current_position.Y - radius };
    const ArcPath arc = ArcPath::calculate({ 10, 10 }, current_position, target_position, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_LE(vSize(arc.end_ - tangent_point), error_margin);
    EXPECT_TRUE(arc.is_clockwise_);
}

TEST_F(ArcPathCalculation_GeneralCreationTest, CorrectTangentPoint_FromNegativeToPositiveValues)
{
    const auto current_position = Point2LL{ -10, -10 };
    const auto target_position = Point2LL{ current_position.X + radius, current_position.Y + 2 * radius };
    const auto tangent_point = Point2LL{ current_position.X + radius, current_position.Y + radius };
    const ArcPath arc = ArcPath::calculate({ -20, -10 }, current_position, target_position, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_LE(vSize(arc.end_ - tangent_point), error_margin);
    EXPECT_FALSE(arc.is_clockwise_);
}

TEST_F(ArcPathCalculation_GeneralCreationTest, CorrectTangentPoint_OnlyNegativeValues)
{
    const auto current_position = Point2LL{ -20'000, -10'000 };
    const auto target_position = Point2LL{ current_position.X - radius, current_position.Y - 2 * radius };
    const auto tangent_point = Point2LL{ current_position.X - radius, current_position.Y - radius };
    const ArcPath arc = ArcPath::calculate({ -10'000, -10'000 }, current_position, target_position, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_LE(vSize(arc.end_ - tangent_point), error_margin);
    EXPECT_FALSE(arc.is_clockwise_);
}


TEST_F(ArcPathCalculation_GeneralCreationTest, DirectionTest_VerticalClockwise)
{
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, { 10'000, 20'000 }, { 15'000, 20'000 }, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_TRUE(arc.is_clockwise_);
}

TEST_F(ArcPathCalculation_GeneralCreationTest, DirectionTest_VerticalCounterClockwise)
{
    const ArcPath arc = ArcPath::calculate({ 15'000, 10'000 }, { 15'000, 20'000 }, { 10'000, 20'000 }, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_FALSE(arc.is_clockwise_);
}

TEST_F(ArcPathCalculation_GeneralCreationTest, DirectionTest_HorizontalClockwise)
{
    const ArcPath arc = ArcPath::calculate({ 10'000, 15'000 }, { 20'000, 15'000 }, { 20'000, 10'000 }, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_TRUE(arc.is_clockwise_);
}

TEST_F(ArcPathCalculation_GeneralCreationTest, DirectionTest_HorizontalCounterClockwise)
{
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, { 20'000, 10'000 }, { 20'000, 15'000 }, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_FALSE(arc.is_clockwise_);
}

TEST_F(ArcPathCalculation_GeneralCreationTest, DirectionTest_DiagonalClockwise)
{
    const ArcPath arc = ArcPath::calculate({ 1000, 1000 }, { 2000, 2000 }, { 2000, 1000 }, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_TRUE(arc.is_clockwise_);
}

TEST_F(ArcPathCalculation_GeneralCreationTest, DirectionTest_DiagonalCounterClockwise)
{
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, { 20'000, 20'000 }, { 10'000, 20'000 }, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_FALSE(arc.is_clockwise_);
}

TEST_F(ArcPathCalculation_GeneralCreationTest, TunaroundTest_SingleTurn)
{
    const auto current_position = Point2LL{ 20'000, 10'000 };
    const auto target_position = Point2LL{ current_position.X + radius, current_position.Y + 2 * radius };
    const double arc_length = (2 * std::numbers::pi * radius) / 4;
    const Velocity high_z_speed = 1.2 * hop_height / (arc_length / xy_speed);
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, current_position, target_position, hop_height, radius, xy_speed, high_z_speed, step_size);
    EXPECT_EQ(arc.n_turns_, 1);
}

TEST_F(ArcPathCalculation_GeneralCreationTest, TunaroundTest_TwoTurnsLowSpeed)
{
    const auto current_position = Point2LL{ 20'000, 20'000 };
    const auto target_position = Point2LL{ current_position.X + radius, current_position.Y + 2 * radius };
    const double circumference = 2 * std::numbers::pi * radius;
    const double arc_length = circumference / 4.0;
    const Velocity low_z_speed = 0.5 * hop_height / (arc_length / xy_speed);
    const double minimal_arc_length = xy_speed * (hop_height / low_z_speed);
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, current_position, target_position, hop_height, radius, xy_speed, low_z_speed, step_size);
    EXPECT_GE(arc.n_turns_, minimal_arc_length / circumference);
}


class ArcPathCalculation_EdgeCaseTest : public testing::Test
{
public:
    const Point2LL previous_position = { 10'000, 10'000 };
    const Point2LL current_position = { 10'000, 20'000 };
    const Point2LL target_position = { 15'000, 20'000 };
    const coord_t hop_height = 1'000;
    const coord_t radius = 10'000;
    const Velocity xy_speed = Velocity{ 100 };
    const Velocity z_speed = Velocity{ 50 };
    const coord_t step_size = 1;
    const AABB3D bounding_box = AABB3D({ 0, 0, 0 }, { 100'000, 100'000, 100'000 });
    const coord_t error_margin = 1;
};


TEST_F(ArcPathCalculation_EdgeCaseTest, ZeroHopHeightTest)
{
    const ArcPath arc = ArcPath::calculate(previous_position, current_position, target_position, 0, radius, xy_speed, z_speed, step_size);
    EXPECT_LT(vSize(current_position - arc.end_) - 2 * radius, error_margin);
    EXPECT_TRUE(arc.is_clockwise_);
}


TEST_F(ArcPathCalculation_EdgeCaseTest, ZeroRadiusTest)
{
    ASSERT_DEATH(ArcPath::calculate(previous_position, current_position, target_position, hop_height, 0, xy_speed, z_speed, step_size), "Arc radius should be larger then zero.");
}


TEST_F(ArcPathCalculation_EdgeCaseTest, BoundingBoxTest_IsInside)
{
    const ArcPath arc = ArcPath::calculate({ 0, 0 }, { 50'000, 50'000 }, { 0, 0 }, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_FALSE(arc.isOutOfBounds(bounding_box)) << "Failed when testing travel moves and arc are valid and inside the printer bounds.";
}

TEST_F(ArcPathCalculation_EdgeCaseTest, BoundingBoxTest_IsOutsideMinY)
{
    const ArcPath arc = ArcPath::calculate({ 50'000, 50'000 }, { 50'000, 0 }, { 70'000, 50'000 }, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_TRUE(arc.isOutOfBounds(bounding_box));
}

TEST_F(ArcPathCalculation_EdgeCaseTest, BoundingBoxTest_IsOutsideMaxY)
{
    const ArcPath arc = ArcPath::calculate({ 50'000, 5000 }, { 50'000, 100'000 }, { 70'000, 50'000 }, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_TRUE(arc.isOutOfBounds(bounding_box));
}

TEST_F(ArcPathCalculation_EdgeCaseTest, BoundingBoxTest_IsOutsideMinX)
{
    const ArcPath arc = ArcPath::calculate({ 50'000, 50'000 }, { 0, 50'000 }, { 50'000, 70'000 }, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_TRUE(arc.isOutOfBounds(bounding_box));
}

TEST_F(ArcPathCalculation_EdgeCaseTest, BoundingBoxTest_IsOutsideMaxX)
{
    const ArcPath arc = ArcPath::calculate({ 50'000, 5000 }, { 100'000, 50'000 }, { 50'000, 70'000 }, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_TRUE(arc.isOutOfBounds(bounding_box));
}

TEST_F(ArcPathCalculation_EdgeCaseTest, BoundingBoxTest_DirectlyOnBounds)
{
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, { 10'000, 100 }, { 11'000, 10'000 }, hop_height, 100, xy_speed, z_speed, step_size);
    EXPECT_TRUE(arc.isOutOfBounds(bounding_box));
}


TEST_F(ArcPathCalculation_EdgeCaseTest, ShortTravelTest_TargetPointOutsideCircle)
{
    const auto current_position = Point2LL{ 20'000, 10'000 };
    const auto target_position = Point2LL{ current_position.X, current_position.Y + 3 * radius };
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, current_position, target_position, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_TRUE((arc.end_ != current_position) && (arc.end_ != target_position)) << "Failed when testing the end point for a travel move which is longer then the circle diameter.";
}

TEST_F(ArcPathCalculation_EdgeCaseTest, ShortTravelTest_TargetPointOnCircle)
{
    const auto current_position = Point2LL{ 20'000, 10'000 };
    const auto target_position = Point2LL{ current_position.X, current_position.Y + 2 * radius };
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, current_position, target_position, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_LT(vSize(arc.end_ - target_position), error_margin) << "Failed when testing the end point for a travel move which is the exact same length as the circle diameter.";
}

TEST_F(ArcPathCalculation_EdgeCaseTest, ShortTravelTest_TargetPointInsideCircle)
{
    const auto current_position = Point2LL{ 20'000, 10'000 };
    const auto target_position = Point2LL{ current_position.X, current_position.Y + 50 };
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, current_position, target_position, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_LT(vSize(arc.end_ - current_position), error_margin) << "Failed when testing the end point for a travel move which is short then the circle diameter.";
}

TEST_F(ArcPathCalculation_EdgeCaseTest, ShortTravelTest_TargetPointIsCurrentPosition)
{
    const auto current_position = Point2LL{ 20'000, 10'000 };
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, current_position, current_position, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_LT(vSize(arc.end_ - current_position), error_margin);
    EXPECT_GE(arc.n_turns_, 1);
}


TEST_F(ArcPathCalculation_EdgeCaseTest, ColinearLineTest_LeftToRightLine)
{
    const auto current_position = Point2LL{ 15'000, 10'000 };
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, current_position, { 30'000, 10'000 }, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_LT(vSize(arc.end_ - current_position), error_margin);
    EXPECT_TRUE(arc.is_clockwise_);
    EXPECT_NEAR(arc.circle_center_.X, current_position.X, error_margin);
    EXPECT_NEAR(arc.circle_center_.Y, current_position.Y - radius, error_margin);
}

TEST_F(ArcPathCalculation_EdgeCaseTest, ColinearLineTest_LeftToRightLineCurrentIsCenter)
{
    const auto current_position = Point2LL{ 20'000, 10'000 };
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, current_position, { 30'000, 10'000 }, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_LT(vSize(arc.end_ - current_position), error_margin);
    EXPECT_TRUE(arc.is_clockwise_);
    EXPECT_NEAR(arc.circle_center_.X, current_position.X, error_margin);
    EXPECT_NEAR(arc.circle_center_.Y, current_position.Y - radius, error_margin);
}

TEST_F(ArcPathCalculation_EdgeCaseTest, ColinearLineTest_RightToLeftLine)
{
    const auto current_position = Point2LL{ 15'000, 10'000 };
    const ArcPath arc = ArcPath::calculate({ 30'000, 10'000 }, current_position, { 10'000, 10'000 }, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_LT(vSize(arc.end_ - current_position), error_margin);
    EXPECT_TRUE(arc.is_clockwise_);
    EXPECT_NEAR(arc.circle_center_.X, current_position.X, error_margin);
    EXPECT_NEAR(arc.circle_center_.Y, current_position.Y + radius, error_margin);
}

TEST_F(ArcPathCalculation_EdgeCaseTest, ColinearLineTest_OppositeDirections)
{
    const auto current_position = Point2LL{ 10'000, 10'000 };
    const auto target_position = Point2LL{ current_position.X + radius, current_position.Y };
    const auto previous_position = Point2LL{ current_position.X + radius / 3, current_position.Y };
    const ArcPath arc = ArcPath::calculate(previous_position, current_position, target_position, hop_height, radius, xy_speed, z_speed, step_size);
    const auto target_tangent = Point2LL{ arc.circle_center_.X + radius, arc.circle_center_.Y };
    const auto target_circle_center = Point2LL{ current_position.X, current_position.Y + radius };
    EXPECT_LE(vSize(arc.circle_center_ - target_circle_center), error_margin);
    EXPECT_LE(vSize(arc.end_ - target_tangent), error_margin);
    EXPECT_TRUE(arc.is_clockwise_);
}

TEST_F(ArcPathCalculation_EdgeCaseTest, ColinearLineTest_BottomToTopLine)
{
    const auto current_position = Point2LL{ 10'000, 15'000 };
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, current_position, { 10'000, 30'000 }, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_LT(vSize(arc.end_ - current_position), error_margin);
    EXPECT_TRUE(arc.is_clockwise_);
    EXPECT_NEAR(arc.circle_center_.Y, current_position.Y, error_margin);
    EXPECT_NEAR(arc.circle_center_.X, current_position.X + radius, error_margin);
}

TEST_F(ArcPathCalculation_EdgeCaseTest, ColinearLineTest_TopToBottomLine)
{
    const auto current_position = Point2LL{ 10'000, 15'000 };
    const ArcPath arc = ArcPath::calculate({ 10'000, 30'000 }, current_position, { 10'000, 10'000 }, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_LT(vSize(arc.end_ - current_position), error_margin);
    EXPECT_TRUE(arc.is_clockwise_);
    EXPECT_NEAR(arc.circle_center_.Y, current_position.Y, error_margin);
    EXPECT_NEAR(arc.circle_center_.X, current_position.X - radius, error_margin);
}

TEST_F(ArcPathCalculation_EdgeCaseTest, ColinearLineTest_DiagonalRisingLineLeftToRight)
{
    const auto current_position = Point2LL{ 15'000, 15'000 };
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, current_position, { 30'000, 30'000 }, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_LT(vSize(arc.end_ - current_position), error_margin);
    EXPECT_TRUE(arc.is_clockwise_);
    const coord_t xy_offset = radius * std::sin(std::numbers::pi / 4);
    EXPECT_NEAR(arc.circle_center_.Y, current_position.Y - xy_offset, error_margin);
    EXPECT_NEAR(arc.circle_center_.X, current_position.X + xy_offset, error_margin);
}

TEST_F(ArcPathCalculation_EdgeCaseTest, ColinearLineTest_DiagonalRisingLineRightToLeft)
{
    const auto current_position = Point2LL{ 15'000, 15'000 };
    const ArcPath arc = ArcPath::calculate({ 30'000, 30'000 }, current_position, { 10'000, 10'000 }, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_LT(vSize(arc.end_ - current_position), error_margin);
    EXPECT_TRUE(arc.is_clockwise_);
    const coord_t xy_offset = radius * std::sin(std::numbers::pi / 4);
    EXPECT_NEAR(arc.circle_center_.Y, current_position.Y + xy_offset, error_margin);
    EXPECT_NEAR(arc.circle_center_.X, current_position.X - xy_offset, error_margin);
}


TEST_F(ArcPathCalculation_EdgeCaseTest, ColinearAndHorizontalAndInRadiusDistance)
{
    const auto current_position = Point2LL{ 20'000, 10'000 };
    const auto target_position = Point2LL{ current_position.X + 2 * radius, current_position.Y };
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, current_position, target_position, hop_height, radius, xy_speed, z_speed, step_size);
    EXPECT_LT(vSize(arc.end_ - current_position), error_margin);
    EXPECT_TRUE(arc.is_clockwise_);
}

TEST_F(ArcPathCalculation_EdgeCaseTest, ColinearAndZeroRadiusTest)
{
    const auto current_position = Point2LL{ 20'000, 10'000 };
    const auto target_position = Point2LL{ current_position.X + 2 * radius, current_position.Y };
    ASSERT_DEATH(ArcPath::calculate({ 10'000, 10'000 }, current_position, target_position, hop_height, 0, xy_speed, z_speed, step_size), "Arc radius should be larger then zero.");
}

TEST_F(ArcPathCalculation_EdgeCaseTest, ColinearAndOutOfBoundingBox)
{
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, { 10'000, 10 }, { 10'000, 5 }, hop_height, 100, xy_speed, z_speed, step_size);
    EXPECT_TRUE(arc.isOutOfBounds(bounding_box));
}

TEST_F(ArcPathCalculation_EdgeCaseTest, OutOfBoundingBoxAndInCicleRadius)
{
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, { 10'000, 100 }, { 11'000, 150 }, hop_height, 200, xy_speed, z_speed, step_size);
    EXPECT_TRUE(arc.isOutOfBounds(bounding_box));
}

class ArcPathCalculation_DiscretizationTest : public testing::Test
{
public:
    const Point2LL previous_position = { 0, 0 };
    const Point2LL current_position = { 80'000, 80'000 };
    const Point2LL target_position = { 10'000, 10'000 };
    const coord_t z_height = 10;
    const coord_t hop_height = 10;
    const coord_t radius = 1'000;
    const Velocity xy_speed = Velocity{ 100 };
    const Velocity z_speed = Velocity{ 50 };
    const coord_t step_size = 100;
    const coord_t error_margin = 1;
};

TEST_F(ArcPathCalculation_DiscretizationTest, CorrectRadius)
{
    const ArcPath arc = ArcPath::calculate(previous_position, current_position, target_position, hop_height, radius, xy_speed, z_speed, step_size);
    const std::vector<std::pair<Point3LL, Velocity>> arc_points = arc.getDiscreteArc(z_height);
    for (size_t idx = 0; idx < arc_points.size(); ++idx)
    {
        EXPECT_LE(vSize(Point2LL{ arc_points[idx].first.x_, arc_points[idx].first.y_ } - arc.circle_center_) - radius, error_margin)
            << "The " << idx << "th point in the arc had a wrong distance to the origin";
    }
}

TEST_F(ArcPathCalculation_DiscretizationTest, CorrectOrdering)
{
    const coord_t step_size = 100;
    const ArcPath arc = ArcPath::calculate(previous_position, current_position, target_position, hop_height, radius, xy_speed, z_speed, step_size);
    const std::vector<std::pair<Point3LL, Velocity>> arc_points = arc.getDiscreteArc(z_height);
    double last_distance = -error_margin;
    bool sign_switch = false;
    for (auto& pt : arc_points)
    {
        coord_t distance_to_start = vSize(current_position - Point2LL{ pt.first.x_, pt.first.y_ });
        if (! sign_switch && distance_to_start <= last_distance)
        {
            last_distance += error_margin;
            sign_switch = true;
        }
        if (! sign_switch)
        {
            EXPECT_GT(distance_to_start, last_distance);
            last_distance = distance_to_start - error_margin;
        }
        else
        {
            EXPECT_LT(distance_to_start, last_distance);
            last_distance = distance_to_start + error_margin;
        }
    }
}

TEST_F(ArcPathCalculation_DiscretizationTest, ZeroHopHeightTest)
{
    const ArcPath arc = ArcPath::calculate(previous_position, current_position, target_position, 0, radius, xy_speed, z_speed, step_size);
    std::vector<std::pair<Point3LL, Velocity>> arc_points = arc.getDiscreteArc(z_height);

    for (auto& pt : arc_points)
    {
        EXPECT_NEAR(pt.first.z_, z_height, error_margin);
    }
}


TEST_F(ArcPathCalculation_DiscretizationTest, ShortTravelTest_TargetPointOnCircle)
{
    const auto current_position = Point2LL{ 20'000, 10'000 };
    const auto target_position = Point2LL{ current_position.X, current_position.Y + 2 * radius };
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, current_position, target_position, hop_height, radius, xy_speed, z_speed, step_size);
    std::vector<std::pair<Point3LL, Velocity>> arc_points = arc.getDiscreteArc(z_height);
    EXPECT_NEAR(vSize(Point2LL{ arc_points[0].first.x_, arc_points[0].first.y_ } - current_position), step_size, error_margin)
        << "Fails when checking that the first point should be step_size far away from the current position.";
    EXPECT_LT(vSize(Point2LL{ arc_points.back().first.x_, arc_points.back().first.y_ } - target_position), error_margin)
        << "Fails when checking that the last point should be the target position.";
    EXPECT_GE(arc_points.size(), 2) << "Fails when checking that there should be at 3 points in the discritzed arc.";
}

TEST_F(ArcPathCalculation_DiscretizationTest, ShortTravelTest_TargetPointInsideCircle)
{
    const auto current_position = Point2LL{ 20'000, 10'000 };
    const auto target_position = Point2LL{ current_position.X, current_position.Y + radius };
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, current_position, target_position, hop_height, radius, xy_speed, z_speed, step_size);
    std::vector<std::pair<Point3LL, Velocity>> arc_points = arc.getDiscreteArc(z_height);
    EXPECT_NEAR(vSize(Point2LL{ arc_points[0].first.x_, arc_points[0].first.y_ } - current_position), step_size, error_margin)
        << "Fails when checking that the first point should be step_size far away from the current position.";
    EXPECT_LT(vSize(Point2LL{ arc_points.back().first.x_, arc_points.back().first.y_ } - current_position), error_margin)
        << "Fails when checking that the last point should be the current position.";
    EXPECT_GE(arc_points.size(), 2) << "Fails when checking that there should be at 3 points in the discritzed arc.";
}


TEST_F(ArcPathCalculation_DiscretizationTest, TravelendsCloserThanStepSize)
{
    const coord_t low_hop_height = 1;
    const auto current_position = Point2LL{ 10'000, 20'000 };
    const auto target_position = Point2LL{ current_position.X + radius, current_position.Y + radius + 2 };
    const ArcPath arc = ArcPath::calculate(
        { 10'000, 10'000 },
        current_position,
        target_position,
        low_hop_height,
        radius,
        xy_speed,
        Velocity{ 1000 },
        2 * vSize(target_position - current_position));
    std::vector<std::pair<Point3LL, Velocity>> arc_points = arc.getDiscreteArc(z_height);
    EXPECT_EQ(arc_points.size(), 1);
    EXPECT_LT(vSize(Point2LL{ arc_points[0].first.x_, arc_points[0].first.y_ } - arc.end_), error_margin);
}


TEST_F(ArcPathCalculation_DiscretizationTest, SpeedCalculationTest_SingleArc)
{
    const auto current_position = Point2LL{ 20'000, 10'000 };
    const auto target_position = Point2LL{ current_position.X + radius, current_position.Y + 2 * radius };
    const double arc_length = (2 * std::numbers::pi * radius) / 4;
    const coord_t total_distance = arc_length + hop_height;
    const Velocity high_z_speed = 1.2 * hop_height / (arc_length / xy_speed);
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, current_position, target_position, hop_height, radius, xy_speed, high_z_speed, step_size);
    const std::vector<std::pair<Point3LL, Velocity>> arc_points = arc.getDiscreteArc(z_height);

    for (auto& pt : arc_points)
    {
        EXPECT_EQ(arc_points[0].second, pt.second) << "The speed does not stay constant over the arc.";
    }
    EXPECT_NEAR(arc_points[0].second * (arc_length / total_distance), xy_speed, error_margin);
    EXPECT_LE(arc_points[0].second * (hop_height / total_distance), high_z_speed + error_margin);
}

TEST_F(ArcPathCalculation_DiscretizationTest, SpeedCalculationTest_MultipleTurnArounds)
{
    const auto current_position = Point2LL{ 20'000, 10'000 };
    const auto target_position = Point2LL{ current_position.X + radius, current_position.Y + 2 * radius };
    const double circumference = 2 * std::numbers::pi * radius;
    const double arc_length = circumference / 4;
    const Velocity low_z_speed = 0.5 * hop_height / (arc_length / xy_speed);
    const ArcPath arc = ArcPath::calculate({ 10'000, 10'000 }, current_position, target_position, hop_height, radius, xy_speed, low_z_speed, step_size);
    const std::vector<std::pair<Point3LL, Velocity>> arc_points = arc.getDiscreteArc(z_height);
    const coord_t total_distance = arc.n_turns_ * circumference + arc_length + hop_height;

    for (auto& pt : arc_points)
    {
        EXPECT_EQ(arc_points[0].second, pt.second) << "The speed does not stay constant over the arc.";
    }
    EXPECT_NEAR(arc_points[0].second * ((arc.n_turns_ * circumference + arc_length) / total_distance), xy_speed, error_margin);
    EXPECT_LE(arc_points[0].second * (hop_height / total_distance), low_z_speed + error_margin);
}

} // namespace cura
