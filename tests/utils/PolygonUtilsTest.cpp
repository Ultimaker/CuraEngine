// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/polygonUtils.h" // The class under test.

#include <gtest/gtest.h>

#include "geometry/Point2LL.h" // Creating and testing with points.
#include "geometry/Polygon.h" // Creating polygons to test with.
#include "utils/Coord_t.h"

// NOLINTBEGIN(*-magic-numbers)
namespace cura
{

struct MoveInsideParameters
{
    Point2LL close_to;
    coord_t distance;
    Point2LL supposed;

    MoveInsideParameters(Point2LL close_to, const coord_t distance, Point2LL supposed)
        : close_to(close_to)
        , distance(distance)
        , supposed(supposed)
    {
    }
};

class MoveInsideTest : public testing::TestWithParam<MoveInsideParameters>
{
public:
    Polygon test_square;
    Polygon pointy_square;

    void SetUp()
    {
        test_square.emplace_back(0, 0);
        test_square.emplace_back(100, 0);
        test_square.emplace_back(100, 100);
        test_square.emplace_back(0, 100);

        pointy_square.emplace_back(0, 0);
        pointy_square.emplace_back(47, 0);
        pointy_square.emplace_back(50, 80);
        pointy_square.emplace_back(53, 0);
        pointy_square.emplace_back(100, 0);
        pointy_square.emplace_back(100, 100);
        pointy_square.emplace_back(55, 100);
        pointy_square.emplace_back(50, 180);
        pointy_square.emplace_back(45, 100);
        pointy_square.emplace_back(0, 100);
    }
};

TEST_P(MoveInsideTest, MoveInside)
{
    const MoveInsideParameters parameters = GetParam();
    const ClosestPointPolygon cpp = PolygonUtils::findClosest(parameters.close_to, test_square);
    Point2LL result = PolygonUtils::moveInside(cpp, parameters.distance);

    // FIXME: Clean-up message with ftm when CURA-8258 is implemented or when we use C++20
    ASSERT_LE(vSize(result - parameters.supposed), 10) << parameters.close_to << " moved with " << parameters.distance << " micron inside to " << result << " rather than "
                                                       << parameters.supposed << ".\n"
                                                       << "\tPS: dist to boundary computed = " << vSize(cpp.location_ - result)
                                                       << "; vs supposed = " << vSize(cpp.location_ - parameters.supposed) << ".\n"
                                                       << "\tclosest_point = " << cpp.location_ << " at index " << cpp.point_idx_ << ".";
}

TEST_P(MoveInsideTest, MoveInside2)
{
    const MoveInsideParameters parameters = GetParam();
    Shape polys;
    polys.push_back(test_square);
    Point2LL result = parameters.close_to;
    PolygonUtils::moveInside2(polys, result, parameters.distance);
    ASSERT_LE(vSize(result - parameters.supposed), 10) << parameters.close_to << " moved with " << parameters.distance << " micron inside to " << result << "rather than "
                                                       << parameters.supposed << ".";
}

INSTANTIATE_TEST_SUITE_P(
    MoveInsideInstantiation,
    MoveInsideTest,
    testing::Values(
        MoveInsideParameters(Point2LL(110, 110), 28, Point2LL(80, 80)), // Near a corner, moving inside.
        MoveInsideParameters(Point2LL(50, 110), 20, Point2LL(50, 80)), // Near an edge, moving inside.
        MoveInsideParameters(Point2LL(110, 110), -28, Point2LL(120, 120)), // Near a corner, moving outside.
        MoveInsideParameters(Point2LL(50, 110), -20, Point2LL(50, 120)), // Near an edge, moving outside.
        MoveInsideParameters(Point2LL(110, 105), 28, Point2LL(80, 80)), // Near a corner but not exactly diagonal.
        MoveInsideParameters(Point2LL(100, 50), 20, Point2LL(80, 50)), // Starting on the border.
        MoveInsideParameters(Point2LL(80, 50), 20, Point2LL(80, 50)), // Already inside.
        MoveInsideParameters(Point2LL(110, 50), 0, Point2LL(100, 50)), // Not keeping any distance from the border.
        MoveInsideParameters(Point2LL(110, 50), 100000, Point2LL(-99900, 50)) // A very far move.
        ));

TEST_F(MoveInsideTest, cornerEdgeTest)
{
    const Point2LL close_to(110, 100);
    const Point2LL supposed1(80, 80); // Allow two possible values here, since the behaviour for this edge case is not specified.
    const Point2LL supposed2(72, 100);
    constexpr coord_t distance = 28;
    const ClosestPointPolygon cpp = PolygonUtils::findClosest(close_to, test_square);
    const Point2LL result = PolygonUtils::moveInside(cpp, distance);

    constexpr coord_t maximum_error = 10;

    // FIXME: Clean-up message with ftm when CURA-8258 is implemented or when we use C++20
    ASSERT_TRUE(vSize(result - supposed1) <= maximum_error || vSize(result - supposed2) <= maximum_error)
        << close_to << " moved with " << distance << " micron inside to " << result << " rather than " << supposed1 << " or " << supposed2 << ".\n"
        << "\tPS: dist to boundary computed = " << vSize(cpp.location_ - result) << "; vs supposed = " << vSize(cpp.location_ - supposed1) << " or "
        << vSize(cpp.location_ - supposed2) << ".\n"
        << "\tclosest point = " << cpp.location_ << " at index " << cpp.point_idx_ << ".";
}

TEST_F(MoveInsideTest, middleTest)
{
    const Point2LL close_to(50, 50);
    const Point2LL supposed1(80, 50); // Allow four possible values here, since the behaviour for this edge case is not specified.
    const Point2LL supposed2(50, 80);
    const Point2LL supposed3(20, 50);
    const Point2LL supposed4(50, 20);
    constexpr coord_t distance = 20;
    const ClosestPointPolygon cpp = PolygonUtils::findClosest(close_to, test_square);
    const Point2LL result = PolygonUtils::moveInside(cpp, distance);

    constexpr coord_t maximum_error = 10;

    // FIXME: Clean-up message with ftm when CURA-8258 is implemented or when we use C++20
    ASSERT_TRUE(
        vSize(result - supposed1) <= maximum_error || vSize(result - supposed2) <= maximum_error || vSize(result - supposed3) <= maximum_error
        || vSize(result - supposed4) <= maximum_error)
        << close_to << " moved with " << distance << " micron inside to " << result << " rather than " << supposed1 << ", " << supposed2 << ", " << supposed3 << " or " << supposed4
        << ".\n"
        << "\tPS: dist to boundary computed = " << vSize(cpp.location_ - result) << "; vs supposed = " << vSize(cpp.location_ - supposed1) << ", "
        << vSize(cpp.location_ - supposed2) << ", " << vSize(cpp.location_ - supposed3) << " or " << vSize(cpp.location_ - supposed4) << ".\n"
        << "\tclosest point = " << cpp.location_ << " at index " << cpp.point_idx_ << ".";
}

TEST_F(MoveInsideTest, middleTestPenalty)
{
    const Point2LL close_to(50, 50);
    const Point2LL supposed(80, 50);
    const Point2LL preferred_dir(120, 60);
    constexpr coord_t distance = 20;
    const ClosestPointPolygon cpp = PolygonUtils::findClosest(
        close_to,
        test_square,
        [preferred_dir](Point2LL candidate)
        {
            return vSize2(candidate - preferred_dir);
        });
    const Point2LL result = PolygonUtils::moveInside(cpp, distance);

    // FIXME: Clean-up message with ftm when CURA-8258 is implemented or when we use C++20
    ASSERT_LE(vSize(result - supposed), 10) << close_to << " moved with " << distance << " micron inside to " << result << " rather than " << supposed << ".\n"
                                            << "\tPS: dist to boundary computed = " << vSize(cpp.location_ - result) << "; vs supposed = " << vSize(cpp.location_ - supposed)
                                            << ".\n"
                                            << "\tclosest point = " << cpp.location_ << " at index " << cpp.point_idx_ << ".";
}

TEST_F(MoveInsideTest, cornerEdgeTest2)
{
    const Point2LL close_to(110, 100);
    const Point2LL supposed1(80, 80); // Allow two possible values here, since the behaviour for this edge case is not specified.
    const Point2LL supposed2(72, 100);
    constexpr coord_t distance = 28;
    Shape polys;
    polys.push_back(test_square);
    Point2LL result = close_to;
    PolygonUtils::moveInside2(polys, result, distance);

    constexpr coord_t maximum_error = 10;
    ASSERT_TRUE(vSize(result - supposed1) <= maximum_error || vSize(result - supposed2) <= maximum_error)
        << close_to << " moved with " << distance << " micron inside to " << result << " rather than " << supposed1 << " or " << supposed2 << ".";
}

TEST_F(MoveInsideTest, pointyCorner)
{
    const Point2LL from(55, 100); // Above pointy bit.
    Point2LL result(from);
    Shape inside;
    inside.push_back(pointy_square);
    ClosestPointPolygon cpp = PolygonUtils::ensureInsideOrOutside(inside, result, 10);

    ASSERT_NE(cpp.point_idx_, NO_INDEX) << "Couldn't ensure point inside close to " << from << ".";
    ASSERT_NE(cpp.poly_idx_, NO_INDEX) << "Couldn't ensure point inside close to " << from << ".";
    ASSERT_TRUE(inside.inside(result)) << from << " couldn't be moved inside.";
}

TEST_F(MoveInsideTest, pointyCornerFail)
{
    // Should fail with normal moveInside2 (and the like).
    const Point2LL from(55, 170); // Above pointy bit.
    Point2LL result(from);
    Shape inside;
    inside.push_back(pointy_square);

    ClosestPointPolygon cpp = PolygonUtils::moveInside2(inside, result, 10);
    ASSERT_NE(cpp.point_idx_, NO_INDEX) << "Couldn't ensure point inside close to " << from << ".";
    ASSERT_NE(cpp.poly_idx_, NO_INDEX) << "Couldn't ensure point inside close to " << from << ".";
    ASSERT_FALSE(inside.inside(result)) << from << " could be moved inside, while it was designed to fail.";
}

TEST_F(MoveInsideTest, outsidePointyCorner)
{
    const Point2LL from(60, 70); // Above pointy bit.
    Point2LL result(from);
    const Point2LL supposed(50, 70); // 10 below pointy bit.
    Shape inside;
    inside.push_back(pointy_square);

    const ClosestPointPolygon cpp = PolygonUtils::ensureInsideOrOutside(inside, result, -10);
    ASSERT_NE(cpp.point_idx_, NO_INDEX) << "Couldn't ensure point inside close to " << from << ".";
    ASSERT_NE(cpp.poly_idx_, NO_INDEX) << "Couldn't ensure point inside close to " << from << ".";
    ASSERT_TRUE(! inside.inside(result)) << from << " couldn't be moved outside.";
}

TEST_F(MoveInsideTest, outsidePointyCornerFail)
{
    // Should fail with normal moveInside2 (and the like).
    const Point2LL from(60, 70); // Above pointy bit.
    Point2LL result(from);
    const Point2LL supposed(50, 70); // 10 below pointy bit.
    Shape inside;
    inside.push_back(pointy_square);

    const ClosestPointPolygon cpp = PolygonUtils::moveInside2(inside, result, -10);
    ASSERT_NE(cpp.point_idx_, NO_INDEX) << "Couldn't ensure point inside close to " << from << ".";
    ASSERT_NE(cpp.poly_idx_, NO_INDEX) << "Couldn't ensure point inside close to " << from << ".";
    ASSERT_FALSE(! inside.inside(result)) << from << " could be moved outside to " << result << ", while it was designed to fail.";
}

struct FindCloseParameters
{
    Point2LL close_to;
    Point2LL supposed;
    coord_t cell_size;
    std::function<int(Point2LL)>* penalty_function;

    FindCloseParameters(const Point2LL close_to, const Point2LL supposed, const coord_t cell_size, std::function<int(Point2LL)>* penalty_function = nullptr)
        : close_to(close_to)
        , supposed(supposed)
        , cell_size(cell_size)
        , penalty_function(penalty_function)
    {
    }
};

// NOLINTBEGIN(misc-non-private-member-variables-in-classes)
class FindCloseTest : public testing::TestWithParam<FindCloseParameters>
{
public:
    Polygon test_square;

    void SetUp() override
    {
        test_square.emplace_back(0, 0);
        test_square.emplace_back(100, 0);
        test_square.emplace_back(100, 100);
        test_square.emplace_back(0, 100);
    }
};
// NOLINTEND(misc-non-private-member-variables-in-classes)

TEST_P(FindCloseTest, FindClose)
{
    const FindCloseParameters parameters = GetParam();
    Shape polygons;
    polygons.push_back(test_square);
    auto loc_to_line = PolygonUtils::createLocToLineGrid(polygons, parameters.cell_size);

    std::optional<ClosestPointPolygon> cpp;
    if (parameters.penalty_function)
    {
        cpp = PolygonUtils::findClose(parameters.close_to, polygons, *loc_to_line, *parameters.penalty_function);
    }
    else
    {
        cpp = PolygonUtils::findClose(parameters.close_to, polygons, *loc_to_line);
    }

    if (cpp)
    {
        const Point2LL result = cpp->location_;
        ASSERT_LE(vSize(result - parameters.supposed), 10) << "Close to " << parameters.close_to << " we found " << result << " rather than " << parameters.supposed << ".\n";
    }
    else
    {
        FAIL() << "Couldn't find anything close to " << parameters.close_to << " (should have been " << parameters.supposed << ").\n";
    }
}

/*
 * Test penalty function to use with findClose.
 */
std::function<int(Point2LL)> testPenalty(
    [](Point2LL candidate)
    {
        return -vSize2(candidate - Point2LL(50, 100)); // The further from 50, 100, the lower the penalty.
    });

INSTANTIATE_TEST_SUITE_P(
    FindCloseInstantiation,
    FindCloseTest,
    testing::Values(
        FindCloseParameters(Point2LL(110, 110), Point2LL(100, 100), 15), // Near a corner.
        FindCloseParameters(Point2LL(50, 110), Point2LL(50, 100), 15), // Near a side.
        FindCloseParameters(Point2LL(50, 50), Point2LL(50, 0), 60, &testPenalty) // Using a penalty function.
        ));

// NOLINTBEGIN(misc-non-private-member-variables-in-classes)
class PolygonUtilsTest : public testing::Test
{
public:
    Shape test_squares;
    Shape test_line;
    Shape test_line_extra_vertices; // Line that has extra vertices along it that are technically unnecessary.

    PolygonUtilsTest()
    {
        Polygon test_square;
        test_square.emplace_back(0, 0);
        test_square.emplace_back(100, 0);
        test_square.emplace_back(100, 100);
        test_square.emplace_back(0, 100);
        test_squares.push_back(test_square);

        Polygon line;
        line.emplace_back(0, 0);
        line.emplace_back(100, 0);
        test_line.push_back(line);

        Polygon line_extra_vertices;
        line_extra_vertices.emplace_back(100, 0);
        line_extra_vertices.emplace_back(25, 0);
        line_extra_vertices.emplace_back(0, 0);
        line_extra_vertices.emplace_back(75, 0);
        test_line_extra_vertices.push_back(line_extra_vertices);
    }
};

struct GetNextParallelIntersectionParameters
{
    std::optional<Point2LL> predicted;
    Point2LL start_point;
    Point2LL line_to;
    bool forward;
    coord_t dist;

    GetNextParallelIntersectionParameters(const std::optional<Point2LL> predicted, const Point2LL start_point, const Point2LL line_to, const bool forward, const coord_t dist)
        : predicted(predicted)
        , start_point(start_point)
        , line_to(line_to)
        , forward(forward)
        , dist(dist)
    {
    }
};

class GetNextParallelIntersectionTest : public testing::TestWithParam<GetNextParallelIntersectionParameters>
{
public:
    Shape test_squares;

    GetNextParallelIntersectionTest()
    {
        Polygon test_square;
        test_square.emplace_back(0, 0);
        test_square.emplace_back(100, 0);
        test_square.emplace_back(100, 100);
        test_square.emplace_back(0, 100);
        test_squares.push_back(test_square);
    }
};
// NOLINTEND(misc-non-private-member-variables-in-classes)

TEST_P(GetNextParallelIntersectionTest, GetNextParallelIntersection)
{
    const GetNextParallelIntersectionParameters parameters = GetParam();
    const ClosestPointPolygon start = PolygonUtils::findClosest(parameters.start_point, test_squares);
    std::optional<ClosestPointPolygon> computed = PolygonUtils::getNextParallelIntersection(start, parameters.line_to, parameters.dist, parameters.forward);

    ASSERT_EQ(bool(parameters.predicted), bool(computed)) << "An answer was predicted but not computed, or computed but not predicted.";
    if (parameters.predicted)
    {
        ASSERT_LT(vSize(*parameters.predicted - computed->p()), 10) << "Result was " << computed->p() << " while it was predicted to be " << *parameters.predicted << "!";
    }
}

INSTANTIATE_TEST_SUITE_P(
    GetNextParallelIntersectionInstantiation,
    GetNextParallelIntersectionTest,
    testing::Values(
        GetNextParallelIntersectionParameters(Point2LL(0, 40), Point2LL(20, 100), Point2LL(150, 200), true, 35),
        GetNextParallelIntersectionParameters(Point2LL(37, 100), Point2LL(80, 100), Point2LL(150, 200), true, 35),
        GetNextParallelIntersectionParameters(Point2LL(70, 100), Point2LL(20, 100), Point2LL(120, 200), false, 35),
        GetNextParallelIntersectionParameters(Point2LL(0, 0), Point2LL(50, 100), Point2LL(150, 200), true, 35),
        GetNextParallelIntersectionParameters(Point2LL(60, 0), Point2LL(10, 0), Point2LL(-90, -100), true, 35),
        GetNextParallelIntersectionParameters(Point2LL(0, 40), Point2LL(10, 0), Point2LL(-90, -100), false, 35),
        GetNextParallelIntersectionParameters(Point2LL(0, 75), Point2LL(50, 100), Point2LL(150, 100), true, 25),
        GetNextParallelIntersectionParameters(Point2LL(25, 100), Point2LL(50, 100), Point2LL(50, 200), true, 25),
        GetNextParallelIntersectionParameters(std::optional<Point2LL>(), Point2LL(100, 100), Point2LL(200, 200), true, 80),
        GetNextParallelIntersectionParameters(Point2LL(0, 45), Point2LL(5, 100), Point2LL(105, 200), true, 35)));

TEST_F(PolygonUtilsTest, RelativeHammingSquaresOverlap)
{
    ASSERT_EQ(PolygonUtils::relativeHammingDistance(test_squares, test_squares), 0);
}

TEST_F(PolygonUtilsTest, RelativeHammingDisjunct)
{
    Shape shifted_polys = test_squares; // Make a copy.
    shifted_polys[0].translate(Point2LL(200, 0));

    ASSERT_EQ(PolygonUtils::relativeHammingDistance(test_squares, shifted_polys), 1.0);
}

TEST_F(PolygonUtilsTest, RelativeHammingHalfOverlap)
{
    Shape shifted_polys = test_squares; // Make a copy.
    shifted_polys[0].translate(Point2LL(50, 0));

    ASSERT_EQ(PolygonUtils::relativeHammingDistance(test_squares, shifted_polys), 0.5);
}

/*
 * Extra test that is similar to RelativeHammingHalfOverlap, but also shifts in
 * the Y direction to make sure that it's not just working when they are exactly
 * axis-aligned.
 */
TEST_F(PolygonUtilsTest, RelativeHammingQuarterOverlap)
{
    Shape shifted_polys = test_squares; // Make a copy.
    shifted_polys[0].translate(Point2LL(50, 50));

    ASSERT_EQ(PolygonUtils::relativeHammingDistance(test_squares, shifted_polys), 0.75);
}

/*
 * Edge case where one of the polygons is a line but the other is not, and the
 * line is contained within the polygon.
 */
TEST_F(PolygonUtilsTest, RelativeHammingLineSquare)
{
    ASSERT_EQ(PolygonUtils::relativeHammingDistance(test_squares, test_line), 1.0)
        << "The difference between the polygons is 100% because the area of the difference encompasses the area of the one polygon that "
           "has "
           "area.";
}

/*
 * Edge case where one of the polygons is the line but the other is not, and the
 * polygons do not overlap.
 */
TEST_F(PolygonUtilsTest, RelativeHammingLineSquareDisjunct)
{
    test_line[0].translate(Point2LL(0, 200));

    ASSERT_EQ(PolygonUtils::relativeHammingDistance(test_squares, test_line), 1.0);
}

TEST_F(PolygonUtilsTest,
       DISABLED_RelativeHammingLineLine) // Disabled because this fails due to a bug in Clipper of testing points inside a line-polygon.
{
    ASSERT_EQ(PolygonUtils::relativeHammingDistance(test_line, test_line), 0.0);
}

TEST_F(PolygonUtilsTest, RelativeHammingLineLineDisjunct)
{
    Shape shifted_line = test_line; // Make a copy.
    shifted_line[0].translate(Point2LL(0, 1));

    ASSERT_EQ(PolygonUtils::relativeHammingDistance(test_line, test_line), 1.0);
}

TEST_F(PolygonUtilsTest, DISABLED_RelativeHammingLineLineDifferentVerts) // Disabled because this fails due to a bug in Clipper of testing
                                                                         // points inside a line-polygon.
{
    ASSERT_EQ(PolygonUtils::relativeHammingDistance(test_line, test_line_extra_vertices), 0.0) << "Even though the exact vertices are different, the actual outline is the same.";
}

} // namespace cura
// NOLINTEND(*-magic-numbers)
