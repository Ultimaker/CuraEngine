//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include <../src/utils/IntPoint.h> //Creating and testing with points.
#include <../src/utils/polygon.h> //Creating polygons to test with.
#include <../src/utils/polygonUtils.h> //The class under test.

namespace cura
{

struct MoveInsideParameters
{
    Point close_to;
    coord_t distance;
    Point supposed;

    MoveInsideParameters(Point close_to, const coord_t distance, Point supposed)
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
    const ClosestPolygonPoint cpp = PolygonUtils::findClosest(parameters.close_to, test_square);
    Point result = PolygonUtils::moveInside(cpp, parameters.distance);
    ASSERT_LE(vSize(result - parameters.supposed), 10)
        << parameters.close_to << " moved with " << parameters.distance << " micron inside to " << result << " rather than " << parameters.supposed << ".\n"
        << "\tPS: dist to boundary computed = " << vSize(cpp.location - result) << "; vs supposed = " << vSize(cpp.location - parameters.supposed) << ".\n"
        << "\tclosest_point = " << cpp.location << " at index " << cpp.point_idx << ".";
}

TEST_P(MoveInsideTest, MoveInside2)
{
    const MoveInsideParameters parameters = GetParam();
    Polygons polys;
    polys.add(test_square);
    Point result = parameters.close_to;
    PolygonUtils::moveInside2(polys, result, parameters.distance);
    ASSERT_LE(vSize(result - parameters.supposed), 10) << parameters.close_to << " moved with " << parameters.distance << " micron inside to " << result << "rather than " << parameters.supposed << ".";
}

INSTANTIATE_TEST_CASE_P(MoveInsideInstantiation, MoveInsideTest, testing::Values(
    MoveInsideParameters(Point(110, 110), 28, Point(80, 80)), //Near a corner, moving inside.
    MoveInsideParameters(Point(50, 110), 20, Point(50, 80)), //Near an edge, moving inside.
    MoveInsideParameters(Point(110, 110), -28, Point(120, 120)), //Near a corner, moving outside.
    MoveInsideParameters(Point(50, 110), -20, Point(50, 120)), //Near an edge, moving outside.
    MoveInsideParameters(Point(110, 105), 28, Point(80, 80)), //Near a corner but not exactly diagonal.
    MoveInsideParameters(Point(100, 50), 20, Point(80, 50)), //Starting on the border.
    MoveInsideParameters(Point(80, 50), 20, Point(80, 50)), //Already inside.
    MoveInsideParameters(Point(110, 50), 0, Point(100, 50)), //Not keeping any distance from the border.
    MoveInsideParameters(Point(110, 50), 100000, Point(-99900, 50)) //A very far move.
));

TEST_F(MoveInsideTest, cornerEdgeTest)
{
    const Point close_to(110, 100);
    const Point supposed1(80, 80); //Allow two possible values here, since the behaviour for this edge case is not specified.
    const Point supposed2(72, 100);
    constexpr coord_t distance = 28;
    const ClosestPolygonPoint cpp = PolygonUtils::findClosest(close_to, test_square);
    const Point result = PolygonUtils::moveInside(cpp, distance);

    constexpr coord_t maximum_error = 10;
    ASSERT_TRUE(vSize(result - supposed1) <= maximum_error || vSize(result - supposed2) <= maximum_error)
        << close_to << " moved with " << distance << " micron inside to " << result << " rather than " << supposed1 << " or " << supposed2 << ".\n"
        << "\tPS: dist to boundary computed = " << vSize(cpp.location - result) << "; vs supposed = " << vSize(cpp.location - supposed1) << " or " << vSize(cpp.location - supposed2) << ".\n"
        << "\tclosest point = " << cpp.location << " at index " << cpp.point_idx << ".";
}

TEST_F(MoveInsideTest, middleTest)
{
    const Point close_to(50, 50);
    const Point supposed1(80, 50); //Allow four possible values here, since the behaviour for this edge case is not specified.
    const Point supposed2(50, 80);
    const Point supposed3(20, 50);
    const Point supposed4(50, 20);
    constexpr coord_t distance = 20;
    const ClosestPolygonPoint cpp = PolygonUtils::findClosest(close_to, test_square);
    const Point result = PolygonUtils::moveInside(cpp, distance);

    constexpr coord_t maximum_error = 10;
    ASSERT_TRUE(vSize(result - supposed1) <= maximum_error || vSize(result - supposed2) <= maximum_error || vSize(result - supposed3) <= maximum_error || vSize(result - supposed4) <= maximum_error)
        << close_to << " moved with " << distance << " micron inside to " << result << " rather than " << supposed1 << ", " << supposed2 << ", " << supposed3 << " or " << supposed4 << ".\n"
        << "\tPS: dist to boundary computed = " << vSize(cpp.location - result) << "; vs supposed = " << vSize(cpp.location - supposed1) << ", " << vSize(cpp.location - supposed2) << ", " << vSize(cpp.location - supposed3) << " or " << vSize(cpp.location - supposed4) << ".\n"
        << "\tclosest point = " << cpp.location << " at index " << cpp.point_idx << ".";
}

TEST_F(MoveInsideTest, middleTestPenalty)
{
    const Point close_to(50, 50);
    const Point supposed(80, 50); 
    const Point preferred_dir(120, 60);
    constexpr coord_t distance = 20;
    const ClosestPolygonPoint cpp = PolygonUtils::findClosest(close_to, test_square, [preferred_dir](Point candidate) { return vSize2(candidate - preferred_dir); } );
    const Point result = PolygonUtils::moveInside(cpp, distance);

    ASSERT_LE(vSize(result - supposed), 10)
        << close_to << " moved with " << distance << " micron inside to " << result << " rather than " << supposed << ".\n"
        << "\tPS: dist to boundary computed = " << vSize(cpp.location - result) << "; vs supposed = " << vSize(cpp.location - supposed) << ".\n"
        << "\tclosest point = " << cpp.location << " at index " << cpp.point_idx << ".";
}

TEST_F(MoveInsideTest, cornerEdgeTest2)
{
    const Point close_to(110, 100);
    const Point supposed1(80, 80); //Allow two possible values here, since the behaviour for this edge case is not specified.
    const Point supposed2(72, 100);
    constexpr coord_t distance = 28;
    Polygons polys;
    polys.add(test_square);
    Point result = close_to;
    PolygonUtils::moveInside2(polys, result, distance);

    constexpr coord_t maximum_error = 10;
    ASSERT_TRUE(vSize(result - supposed1) <= maximum_error || vSize(result - supposed2) <= maximum_error)
        << close_to << " moved with " << distance << " micron inside to " << result << " rather than " << supposed1 << " or " << supposed2 << ".";
}

TEST_F(MoveInsideTest, pointyCorner)
{
    const Point from(55, 100); //Above pointy bit.
    Point result(from);
    Polygons inside;
    inside.add(pointy_square);
    ClosestPolygonPoint cpp = PolygonUtils::ensureInsideOrOutside(inside, result, 10);

    ASSERT_NE(cpp.point_idx, NO_INDEX) << "Couldn't ensure point inside close to " << from << ".";
    ASSERT_NE(cpp.poly_idx, NO_INDEX) << "Couldn't ensure point inside close to " << from << ".";
    ASSERT_TRUE(inside.inside(result)) << from << " couldn't be moved inside.";
}

TEST_F(MoveInsideTest, pointyCornerFail)
{
    //Should fail with normal moveInside2 (and the like).
    const Point from(55, 170); //Above pointy bit.
    Point result(from);
    Polygons inside;
    inside.add(pointy_square);
    
    ClosestPolygonPoint cpp = PolygonUtils::moveInside2(inside, result, 10);
    ASSERT_NE(cpp.point_idx, NO_INDEX) << "Couldn't ensure point inside close to " << from << ".";
    ASSERT_NE(cpp.poly_idx, NO_INDEX) << "Couldn't ensure point inside close to " << from << ".";
    ASSERT_FALSE(inside.inside(result)) << from << " could be moved inside, while it was designed to fail.";
}

TEST_F(MoveInsideTest, outsidePointyCorner)
{
    const Point from(60, 70); //Above pointy bit.
    Point result(from);
    const Point supposed(50, 70); //10 below pointy bit.
    Polygons inside;
    inside.add(pointy_square);

    const ClosestPolygonPoint cpp = PolygonUtils::ensureInsideOrOutside(inside, result, -10);
    ASSERT_NE(cpp.point_idx, NO_INDEX) << "Couldn't ensure point inside close to " << from << ".";
    ASSERT_NE(cpp.poly_idx, NO_INDEX) << "Couldn't ensure point inside close to " << from << ".";
    ASSERT_TRUE(!inside.inside(result)) << from << " couldn't be moved outside.";
}

TEST_F(MoveInsideTest, outsidePointyCornerFail)
{
    //Should fail with normal moveInside2 (and the like).
    const Point from(60, 70); //Above pointy bit.
    Point result(from);
    const Point supposed(50, 70); //10 below pointy bit.
    Polygons inside;
    inside.add(pointy_square);

    const ClosestPolygonPoint cpp = PolygonUtils::moveInside2(inside, result, -10);
    ASSERT_NE(cpp.point_idx, NO_INDEX) << "Couldn't ensure point inside close to " << from << ".";
    ASSERT_NE(cpp.poly_idx, NO_INDEX) << "Couldn't ensure point inside close to " << from << ".";
    ASSERT_FALSE(!inside.inside(result)) << from << " could be moved outside to " << result << ", while it was designed to fail.";
}

struct FindCloseParameters
{
    Point close_to;
    Point supposed;
    coord_t cell_size;
    std::function<int(Point)>* penalty_function;

    FindCloseParameters(const Point close_to, const Point supposed, const coord_t cell_size, std::function<int(Point)>* penalty_function = nullptr)
    : close_to(close_to)
    , supposed(supposed)
    , cell_size(cell_size)
    , penalty_function(penalty_function)
    {
    }
};

class FindCloseTest : public testing::TestWithParam<FindCloseParameters>
{
public:
    Polygon test_square;

    void SetUp()
    {
        test_square.emplace_back(0, 0);
        test_square.emplace_back(100, 0);
        test_square.emplace_back(100, 100);
        test_square.emplace_back(0, 100);
    }
};

TEST_P(FindCloseTest, FindClose)
{
    const FindCloseParameters parameters = GetParam();
    Polygons polygons;
    polygons.add(test_square);
    auto loc_to_line = PolygonUtils::createLocToLineGrid(polygons, parameters.cell_size);

    std::optional<ClosestPolygonPoint> cpp;
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
        const Point result = cpp->location;
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
std::function<int(Point)> testPenalty([](Point candidate)
{
   return -vSize2(candidate - Point(50, 100)); //The further from 50, 100, the lower the penalty.
});

INSTANTIATE_TEST_CASE_P(FindCloseInstantiation, FindCloseTest, testing::Values(
    FindCloseParameters(Point(110, 110), Point(100, 100), 15), //Near a corner.
    FindCloseParameters(Point(50, 110), Point(50, 100), 15), //Near a side.
    FindCloseParameters(Point(50, 50), Point(50, 0), 60, &testPenalty) //Using a penalty function.
));

class PolygonUtilsTest : public testing::Test
{
public:
    Polygons test_squares;
    Polygons test_line;
    Polygons test_line_extra_vertices; //Line that has extra vertices along it that are technically unnecessary.

    PolygonUtilsTest()
    {
        Polygon test_square;
        test_square.emplace_back(0, 0);
        test_square.emplace_back(100, 0);
        test_square.emplace_back(100, 100);
        test_square.emplace_back(0, 100);
        test_squares.add(test_square);

        Polygon line;
        line.emplace_back(0, 0);
        line.emplace_back(100, 0);
        test_line.add(line);

        Polygon line_extra_vertices;
        line_extra_vertices.emplace_back(100, 0);
        line_extra_vertices.emplace_back(25, 0);
        line_extra_vertices.emplace_back(0, 0);
        line_extra_vertices.emplace_back(75, 0);
        test_line_extra_vertices.add(line_extra_vertices);
    }
};

TEST_F(PolygonUtilsTest, spreadDotsSegment)
{
    std::vector<ClosestPolygonPoint> supposed;
    supposed.emplace_back(Point(50, 0), 0, test_squares[0], 0);
    supposed.emplace_back(Point(100, 0), 1, test_squares[0], 0);
    supposed.emplace_back(Point(100, 50), 1, test_squares[0], 0);

    std::vector<ClosestPolygonPoint> result;
    PolygonUtils::spreadDots(PolygonsPointIndex(&test_squares, 0, 0), PolygonsPointIndex(&test_squares, 0, 2), 3, result);

    ASSERT_EQ(result.size(), supposed.size());
    for (size_t point_idx = 0; point_idx < result.size(); point_idx++)
    {
        EXPECT_EQ(result[point_idx].p(), supposed[point_idx].p());
    }
}

TEST_F(PolygonUtilsTest, spreadDotsFull)
{
    std::vector<ClosestPolygonPoint> supposed;
    supposed.emplace_back(Point(0, 0), 0, test_squares[0], 0);
    supposed.emplace_back(Point(50, 0), 0, test_squares[0], 0);
    supposed.emplace_back(Point(100, 0), 1, test_squares[0], 0);
    supposed.emplace_back(Point(100, 50), 1, test_squares[0], 0);
    supposed.emplace_back(Point(100, 100), 2, test_squares[0], 0);
    supposed.emplace_back(Point(50, 100), 2, test_squares[0], 0);
    supposed.emplace_back(Point(0, 100), 3, test_squares[0], 0);
    supposed.emplace_back(Point(0, 50), 3, test_squares[0], 0);

    std::vector<ClosestPolygonPoint> result;
    PolygonUtils::spreadDots(PolygonsPointIndex(&test_squares, 0, 0), PolygonsPointIndex(&test_squares, 0, 0), 8, result);

    ASSERT_EQ(result.size(), supposed.size());
    for (size_t point_idx = 0; point_idx < result.size(); point_idx++)
    {
        EXPECT_EQ(result[point_idx].p(), supposed[point_idx].p());
    }
}

struct GetNextParallelIntersectionParameters
{
    std::optional<Point> predicted;
    Point start_point;
    Point line_to;
    bool forward;
    coord_t dist;

    GetNextParallelIntersectionParameters(const std::optional<Point> predicted, const Point start_point, const Point line_to, const bool forward, const coord_t dist)
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
    Polygons test_squares;

    GetNextParallelIntersectionTest()
    {
        Polygon test_square;
        test_square.emplace_back(0, 0);
        test_square.emplace_back(100, 0);
        test_square.emplace_back(100, 100);
        test_square.emplace_back(0, 100);
        test_squares.add(test_square);
    }
};

TEST_P(GetNextParallelIntersectionTest, GetNextParallelIntersection)
{
    const GetNextParallelIntersectionParameters parameters = GetParam();
    const ClosestPolygonPoint start = PolygonUtils::findClosest(parameters.start_point, test_squares);
    std::optional<ClosestPolygonPoint> computed = PolygonUtils::getNextParallelIntersection(start, parameters.line_to, parameters.dist, parameters.forward);

    ASSERT_EQ(bool(parameters.predicted), bool(computed)) << "An answer was predicted but not computed, or computed but not predicted.";
    if (parameters.predicted)
    {
        ASSERT_LT(vSize(*parameters.predicted - computed->p()), 10) << "Result was " << computed->p() << " while it was predicted to be " << *parameters.predicted << "!";
    }
}

INSTANTIATE_TEST_CASE_P(GetNextParallelIntersectionInstantiation, GetNextParallelIntersectionTest, testing::Values(
    GetNextParallelIntersectionParameters(Point(0, 40), Point(20, 100), Point(150, 200), true, 35),
    GetNextParallelIntersectionParameters(Point(37, 100), Point(80, 100), Point(150, 200), true, 35),
    GetNextParallelIntersectionParameters(Point(70, 100), Point(20, 100), Point(120, 200), false, 35),
    GetNextParallelIntersectionParameters(Point(0, 0), Point(50, 100), Point(150, 200), true, 35),
    GetNextParallelIntersectionParameters(Point(60, 0), Point(10, 0), Point(-90, -100), true, 35),
    GetNextParallelIntersectionParameters(Point(0, 40), Point(10, 0), Point(-90, -100), false, 35),
    GetNextParallelIntersectionParameters(Point(0, 75), Point(50, 100), Point(150, 100), true, 25),
    GetNextParallelIntersectionParameters(Point(25, 100), Point(50, 100), Point(50, 200), true, 25),
    GetNextParallelIntersectionParameters(std::optional<Point>(), Point(100, 100), Point(200, 200), true, 80),
    GetNextParallelIntersectionParameters(Point(0, 45), Point(5, 100), Point(105, 200), true, 35)
));

TEST_F(PolygonUtilsTest, RelativeHammingSquaresOverlap)
{
    ASSERT_EQ(PolygonUtils::relativeHammingDistance(test_squares, test_squares), 0);
}

TEST_F(PolygonUtilsTest, RelativeHammingDisjunct)
{
    Polygons shifted_polys = test_squares; //Make a copy.
    shifted_polys[0].translate(Point(200, 0));

    ASSERT_EQ(PolygonUtils::relativeHammingDistance(test_squares, shifted_polys), 1.0);
}

TEST_F(PolygonUtilsTest, RelativeHammingHalfOverlap)
{
    Polygons shifted_polys = test_squares; //Make a copy.
    shifted_polys[0].translate(Point(50, 0));

    ASSERT_EQ(PolygonUtils::relativeHammingDistance(test_squares, shifted_polys), 0.5);
}

/*
 * Extra test that is similar to RelativeHammingHalfOverlap, but also shifts in
 * the Y direction to make sure that it's not just working when they are exactly
 * axis-aligned.
 */
TEST_F(PolygonUtilsTest, RelativeHammingQuarterOverlap)
{
    Polygons shifted_polys = test_squares; //Make a copy.
    shifted_polys[0].translate(Point(50, 50));

    ASSERT_EQ(PolygonUtils::relativeHammingDistance(test_squares, shifted_polys), 0.75);
}

/*
 * Edge case where one of the polygons is a line but the other is not, and the
 * line is contained within the polygon.
 */
TEST_F(PolygonUtilsTest, RelativeHammingLineSquare)
{
    ASSERT_EQ(PolygonUtils::relativeHammingDistance(test_squares, test_line), 1.0) << "The difference between the polygons is 100% because the area of the difference encompasses the area of the one polygon that has area.";
}

/*
 * Edge case where one of the polygons is the line but the other is not, and the
 * polygons do not overlap.
 */
TEST_F(PolygonUtilsTest, RelativeHammingLineSquareDisjunct)
{
    test_line[0].translate(Point(0, 200));

    ASSERT_EQ(PolygonUtils::relativeHammingDistance(test_squares, test_line), 1.0);
}

TEST_F(PolygonUtilsTest, DISABLED_RelativeHammingLineLine) //Disabled because this fails due to a bug in Clipper of testing points inside a line-polygon.
{
    ASSERT_EQ(PolygonUtils::relativeHammingDistance(test_line, test_line), 0.0);
}

TEST_F(PolygonUtilsTest, RelativeHammingLineLineDisjunct)
{
    Polygons shifted_line = test_line; //Make a copy.
    shifted_line[0].translate(Point(0, 1));

    ASSERT_EQ(PolygonUtils::relativeHammingDistance(test_line, test_line), 1.0);
}

TEST_F(PolygonUtilsTest, DISABLED_RelativeHammingLineLineDifferentVerts) //Disabled because this fails due to a bug in Clipper of testing points inside a line-polygon.
{
    ASSERT_EQ(PolygonUtils::relativeHammingDistance(test_line, test_line_extra_vertices), 0.0) << "Even though the exact vertices are different, the actual outline is the same.";
}

}
