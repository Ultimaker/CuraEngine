//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

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


}
