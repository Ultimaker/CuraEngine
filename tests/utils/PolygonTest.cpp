//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <../src/utils/polygon.h> //The class under test.

namespace cura
{

class PolygonTest: public testing::Test
{
public:
    Polygon test_square;
    Polygon pointy_square;
    Polygon triangle;
    Polygon clipper_bug;
    Polygon clockwise_large;
    Polygon clockwise_small;
    Polygons clockwise_donut;
    Polygon line;

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

        triangle.emplace_back(100, 0);
        triangle.emplace_back(300, 0);
        triangle.emplace_back(200, 100);

        clipper_bug.emplace_back(107347, 120836);
        clipper_bug.emplace_back(107309, 120910);
        clipper_bug.emplace_back(107158, 120960);
        clipper_bug.emplace_back(106760, 120839);
        clipper_bug.emplace_back(106570, 120831);

        clockwise_large.emplace_back(-100, -100);
        clockwise_large.emplace_back(-100, 100);
        clockwise_large.emplace_back(100, 100);
        clockwise_large.emplace_back(100, -100);

        clockwise_small.emplace_back(-50, -50);
        clockwise_small.emplace_back(-50, 50);
        clockwise_small.emplace_back(50, 50);
        clockwise_small.emplace_back(50, -50);

        Polygons outer, inner;
        outer.add(clockwise_large);
        inner.add(clockwise_small);
        clockwise_donut = outer.difference(inner);

        line.emplace_back(0, 0);
        line.emplace_back(100, 0);
    }
};

TEST_F(PolygonTest, polygonOffsetTest)
{
    Polygons test_squares;
    test_squares.add(test_square);
    const Polygons expanded = test_squares.offset(25);
    const coord_t expanded_length = expanded.polygonLength();

    Polygons square_hole;
    PolygonRef square_inverted = square_hole.newPoly();
    for (int i = test_square.size() - 1; i >= 0; i--)
    {
        square_inverted.add(test_square[i]);
    }
    const Polygons contracted = square_hole.offset(25);
    const coord_t contracted_length = contracted.polygonLength();

    ASSERT_NEAR(expanded_length, contracted_length, 5) << "Offset on outside poly is different from offset on inverted poly!";
}

TEST_F(PolygonTest, polygonOffsetBugTest)
{
    Polygons polys;
    polys.add(clipper_bug);
    const Polygons offsetted = polys.offset(-20);

    for (const ConstPolygonRef poly : offsetted)
    {
        for (const Point& p : poly)
        {
            ASSERT_TRUE(polys.inside(p)) << "A negative offset should move the point towards the inside!";
        }
    }
}

TEST_F(PolygonTest, isOutsideTest)
{
    Polygons test_triangle;
    test_triangle.add(triangle);

    EXPECT_FALSE(test_triangle.inside(Point(0, 100))) << "Left point should be outside the triangle.";
    EXPECT_FALSE(test_triangle.inside(Point(100, 100))) << "Middle left point should be outside the triangle.";
    EXPECT_FALSE(test_triangle.inside(Point(300, 100))) << "Middle right point should be outside the triangle.";
    EXPECT_FALSE(test_triangle.inside(Point(500, 100))) << "Right point should be outside the triangle.";
    EXPECT_FALSE(test_triangle.inside(Point(100, 200))) << "Above point should be outside the triangle.";
    EXPECT_FALSE(test_triangle.inside(Point(100, -100))) << "Below point should be outside the triangle.";
}

TEST_F(PolygonTest, isInsideTest)
{
    Polygons test_polys;
    PolygonRef poly = test_polys.newPoly();
    poly.add(Point(82124,98235));
    poly.add(Point(83179,98691));
    poly.add(Point(83434,98950));
    poly.add(Point(82751,99026));
    poly.add(Point(82528,99019));
    poly.add(Point(81605,98854));
    poly.add(Point(80401,98686));
    poly.add(Point(79191,98595));
    poly.add(Point(78191,98441));
    poly.add(Point(78998,98299));
    poly.add(Point(79747,98179));
    poly.add(Point(80960,98095));

    EXPECT_TRUE(test_polys.inside(Point(78315, 98440))) << "Point should be inside the polygons!";
}

TEST_F(PolygonTest, isOnBorderTest)
{
    Polygons test_triangle;
    test_triangle.add(triangle);

    EXPECT_FALSE(test_triangle.inside(Point(200, 0), false)) << "Point is on the bottom edge of the triangle.";
    EXPECT_TRUE(test_triangle.inside(Point(200, 0), true)) << "Point is on the bottom edge of the triangle.";
    EXPECT_FALSE(test_triangle.inside(Point(150, 50), false)) << "Point is on a diagonal side of the triangle.";
    EXPECT_TRUE(test_triangle.inside(Point(150, 50), true)) << "Point is on a diagonal side of the triangle.";
}

TEST_F(PolygonTest, DISABLED_isInsideLineTest) //Disabled because this fails due to a bug in Clipper.
{
    Polygons polys;
    polys.add(line);

    EXPECT_FALSE(polys.inside(Point(50, 0), false)) << "Should be outside since it is on the border and border is considered outside.";
    EXPECT_TRUE(polys.inside(Point(50, 0), true)) << "Should be inside since it is on the border and border is considered inside.";
}

TEST_F(PolygonTest, splitIntoPartsWithHoleTest)
{
    const std::vector<PolygonsPart> parts = clockwise_donut.splitIntoParts();

    EXPECT_EQ(parts.size(), 1) << "Difference between two polygons should be one PolygonsPart!";
}

TEST_F(PolygonTest, differenceContainsOriginalPointTest)
{
    const PolygonsPart part = clockwise_donut.splitIntoParts()[0];
    const ConstPolygonRef outer = part.outerPolygon();
    EXPECT_NE(std::find(outer.begin(), outer.end(), clockwise_large[0]), outer.end()) << "Outer vertex must be in polygons difference.";
    const ConstPolygonRef inner = part[1];
    EXPECT_NE(std::find(inner.begin(), inner.end(), clockwise_small[0]), inner.end()) << "Inner vertex must be in polygons difference.";
}

TEST_F(PolygonTest, differenceClockwiseTest)
{
    const PolygonsPart part = clockwise_donut.splitIntoParts()[0];

    const ConstPolygonRef outer = part.outerPolygon();
    //Apply the shoelace formula to determine surface area. If it's negative, the polygon is counterclockwise.
    coord_t area = 0;
    for (size_t point_index = 0; point_index < outer.size(); point_index++)
    {
        const size_t next_index = (point_index + 1) % outer.size();
        const Point point = outer[point_index];
        const Point next = outer[next_index];
        area += (next.X - point.X) * (point.Y + next.Y);
    }
    EXPECT_LT(area, 0) << "Outer polygon should be counter-clockwise.";

    const ConstPolygonRef inner = part[1];
    area = 0;
    for (size_t point_index = 0; point_index < inner.size(); point_index++)
    {
        const size_t next_index = (point_index + 1) % inner.size();
        const Point point = inner[point_index];
        const Point next = inner[next_index];
        area += (next.X - point.X) * (point.Y + next.Y);
    }
    EXPECT_GT(area, 0) << "Inner polygon should be clockwise.";
}

TEST_F(PolygonTest, getEmptyHolesTest)
{
    const Polygons holes = clockwise_donut.getEmptyHoles();

    ASSERT_EQ(holes.size(), 1);
    ASSERT_EQ(holes[0].size(), clockwise_small.size()) << "Empty hole should have the same amount of vertices as the original polygon.";
    for (size_t point_index = 0; point_index < holes[0].size(); point_index++)
    {
        EXPECT_EQ(holes[0][point_index], clockwise_small[point_index]) << "Coordinates of the empty hole must be the same as the original polygon.";
    }
}

TEST_F(PolygonTest, simplifyCircle)
{
    Polygons circle_polygons;
    PolygonRef circle = circle_polygons.newPoly();
    constexpr coord_t radius = 100000;
    constexpr double segment_length = 1000;
    constexpr double tau = 6.283185307179586476925286766559; //2 * pi.
    constexpr double increment = segment_length / radius; //Segments of 1000 units.
    for (double angle = 0; angle < tau; angle += increment)
    {
        circle.add(Point(std::cos(angle) * radius, std::sin(angle) * radius));
    }

    constexpr coord_t minimum_segment_length = segment_length + 10;
    circle_polygons.simplify(minimum_segment_length, 999999999); //With segments of 1000, we need to remove exactly half of the vertices to meet the requirement that all segments are >1010.
    constexpr coord_t maximum_segment_length = segment_length * 2 + 20; //+20 for some error margin due to rounding.

    for (size_t point_index = 1; point_index < circle.size() - 1; point_index++) //Don't check the last vertex. Due to odd-numbered vertices it has to be shorter than the minimum.
    {
        coord_t segment_length = vSize(circle[point_index % circle.size()] - circle[point_index - 1]);
        ASSERT_GE(segment_length, minimum_segment_length) << "Segment " << (point_index - 1) << " - " << point_index << " is too short!";
        ASSERT_LE(segment_length, maximum_segment_length) << "Segment " << (point_index - 1) << " - " << point_index << " is too long!";
    }
}

TEST_F(PolygonTest, simplifyZigzag)
{
    //Tests a zigzag line: /\/\/\/\/\/\/
    //If all line segments are short, they can all be removed and turned into one long line: -------------------
    Polygons zigzag_polygons;
    PolygonRef zigzag = zigzag_polygons.newPoly();
    constexpr coord_t segment_length = 1000;
    const coord_t y_increment = segment_length / std::sqrt(2);
    coord_t x = y_increment / 2;

    for (size_t i = 0; i < 100; i++)
    {
        zigzag.add(Point(x, i * y_increment));
        x = 0 - x;
    }

    constexpr coord_t maximum_error = 2 * segment_length * segment_length + 100; //Squared offset from baseline (and some margin for rounding).
    zigzag_polygons.simplify(segment_length + 10, maximum_error);

    ASSERT_LE(zigzag.size(), 5) << "Zigzag should be removed since the total error compensates with each zag.";
}

TEST_F(PolygonTest, simplifyLimitedLength)
{
    //Generate a spiral with segments that gradually increase in length.
    Polygons spiral_polygons;
    PolygonRef spiral = spiral_polygons.newPoly();
    spiral.add(Point());

    coord_t segment_length = 1000;
    double angle = 0;
    Point last_position;

    for (size_t i = 0; i < 10; i++)
    {
        const coord_t dx = std::cos(angle) * segment_length;
        const coord_t dy = std::sin(angle) * segment_length;
        last_position += Point(dx, dy);
        spiral.add(last_position);
        segment_length += 100;
        angle += 0.1;
    }

    spiral_polygons.simplify(1550, 999999999); //Remove segments smaller than 1550 (infinite area error).

    ASSERT_EQ(spiral.size(), 11 - 3) << "Should merge segments of length 1100 with 1200, 1300 with 1400 and first with last.";
}

TEST_F(PolygonTest, simplifyLimitedError)
{
    //Generate a square spiral with increasingly large corners until the area exceeds the limit.
    Polygons spiral_polygons;
    PolygonRef spiral = spiral_polygons.newPoly();
    spiral.add(Point());

    //Generate a square spiral, 90 degree corners to make it easy to compute the area loss while retaining a positive area per corner.
    coord_t segment_length = 1000;
    Point last_position;
    double angle = 0;
    for (size_t i = 0; i < 10; i++)
    {
        const coord_t dx = std::cos(angle) * segment_length;
        const coord_t dy = std::sin(angle) * segment_length;
        last_position += Point(dx, dy);
        spiral.add(last_position);
        segment_length += 100;
        angle += M_PI / 2;
    }

    //We want it to not merge the lines 1400 and 1500 any more, but do merge all lines before it.
    //Take the area of the 1400 by 1500 and plug it into the formula for the height to get at the baseline height, which is our allowed error.
    constexpr coord_t area = 1400 * 1500 / 2;
    const coord_t diagonal_length = std::sqrt(1400 * 1400 + 1500 * 1500); //Pythagoras.
    //A = 0.5 * b * h. diagonal_length is the base line in this case.
    //2A = b * h
    //2A / b = h
    const coord_t height = 4 * area / diagonal_length; //Error of the first vertex we want to keep, so we must set the limit to something slightly lower than this.
    spiral_polygons.simplify(999999999, height - 10);

    using namespace testing;
    EXPECT_THAT(spiral.size(), AllOf(Ge(11 - 5), Le(11 - 4))) << "Should merge segments of length 1000 through 1400 and (optionally) first with last.";
}

}
