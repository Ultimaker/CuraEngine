// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/polygon.h" // The class under test.

#include <gtest/gtest.h>

#include "utils/Coord_t.h"
#include "utils/SVG.h" // helper functions
#include "utils/polygonUtils.h" // helper functions

// NOLINTBEGIN(*-magic-numbers)
namespace cura
{

// NOLINTBEGIN(misc-non-private-member-variables-in-classes)
class PolygonTest : public testing::Test
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
    Polygon small_area;

    void SetUp() override
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

        Polygons outer;
        Polygons inner;
        outer.add(clockwise_large);
        inner.add(clockwise_small);
        clockwise_donut = outer.difference(inner);

        line.emplace_back(0, 0);
        line.emplace_back(100, 0);

        small_area.emplace_back(0, 0);
        small_area.emplace_back(10, 0);
        small_area.emplace_back(10, 10);
        small_area.emplace_back(0, 10);
    }
    void twoPolygonsAreEqual(Polygons& polygon1, Polygons& polygon2) const
    {
        auto poly_cmp = [](const ClipperLib::Path& a, const ClipperLib::Path& b)
        {
            return std::lexicographical_compare(
                a.begin(),
                a.end(),
                b.begin(),
                b.end(),
                [](const Point2LL& p1, const Point2LL& p2)
                {
                    return p1 < p2;
                });
        };
        std::sort(polygon1.begin(), polygon1.end(), poly_cmp);
        std::sort(polygon2.begin(), polygon2.end(), poly_cmp);

        std::vector<ClipperLib::Path> difference;
        std::set_difference(polygon1.begin(), polygon1.end(), polygon2.begin(), polygon2.end(), std::back_inserter(difference), poly_cmp);
        ASSERT_TRUE(difference.empty()) << "Paths in polygon1 not found in polygon2:" << difference;

        difference.clear();
        std::set_difference(polygon2.begin(), polygon2.end(), polygon1.begin(), polygon1.end(), std::back_inserter(difference), poly_cmp);
        ASSERT_TRUE(difference.empty()) << "Paths in polygon2 not found in polygon1:" << difference;
    }
};
// NOLINTEND(misc-non-private-member-variables-in-classes)

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
        for (const Point2LL& p : poly)
        {
            ASSERT_TRUE(polys.inside(p)) << "A negative offset should move the point towards the inside!";
        }
    }
}

TEST_F(PolygonTest, isOutsideTest)
{
    Polygons test_triangle;
    test_triangle.add(triangle);

    EXPECT_FALSE(test_triangle.inside(Point2LL(0, 100))) << "Left point should be outside the triangle.";
    EXPECT_FALSE(test_triangle.inside(Point2LL(100, 100))) << "Middle left point should be outside the triangle.";
    EXPECT_FALSE(test_triangle.inside(Point2LL(300, 100))) << "Middle right point should be outside the triangle.";
    EXPECT_FALSE(test_triangle.inside(Point2LL(500, 100))) << "Right point should be outside the triangle.";
    EXPECT_FALSE(test_triangle.inside(Point2LL(100, 200))) << "Above point should be outside the triangle.";
    EXPECT_FALSE(test_triangle.inside(Point2LL(100, -100))) << "Below point should be outside the triangle.";
}

TEST_F(PolygonTest, isInsideTest)
{
    Polygons test_polys;
    PolygonRef poly = test_polys.newPoly();
    poly.add(Point2LL(82124, 98235));
    poly.add(Point2LL(83179, 98691));
    poly.add(Point2LL(83434, 98950));
    poly.add(Point2LL(82751, 99026));
    poly.add(Point2LL(82528, 99019));
    poly.add(Point2LL(81605, 98854));
    poly.add(Point2LL(80401, 98686));
    poly.add(Point2LL(79191, 98595));
    poly.add(Point2LL(78191, 98441));
    poly.add(Point2LL(78998, 98299));
    poly.add(Point2LL(79747, 98179));
    poly.add(Point2LL(80960, 98095));

    EXPECT_TRUE(test_polys.inside(Point2LL(78315, 98440))) << "Point should be inside the polygons!";
}

TEST_F(PolygonTest, isOnBorderTest)
{
    Polygons test_triangle;
    test_triangle.add(triangle);

    EXPECT_FALSE(test_triangle.inside(Point2LL(200, 0), false)) << "Point is on the bottom edge of the triangle.";
    EXPECT_TRUE(test_triangle.inside(Point2LL(200, 0), true)) << "Point is on the bottom edge of the triangle.";
    EXPECT_FALSE(test_triangle.inside(Point2LL(150, 50), false)) << "Point is on a diagonal side of the triangle.";
    EXPECT_TRUE(test_triangle.inside(Point2LL(150, 50), true)) << "Point is on a diagonal side of the triangle.";
}

TEST_F(PolygonTest, DISABLED_isInsideLineTest) // Disabled because this fails due to a bug in Clipper.
{
    Polygons polys;
    polys.add(line);

    EXPECT_FALSE(polys.inside(Point2LL(50, 0), false)) << "Should be outside since it is on the border and border is considered outside.";
    EXPECT_TRUE(polys.inside(Point2LL(50, 0), true)) << "Should be inside since it is on the border and border is considered inside.";
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
    // Apply the shoelace formula to determine surface area. If it's negative, the polygon is counterclockwise.
    coord_t area = 0;
    for (size_t point_index = 0; point_index < outer.size(); point_index++)
    {
        const size_t next_index = (point_index + 1) % outer.size();
        const Point2LL point = outer[point_index];
        const Point2LL next = outer[next_index];
        area += (next.X - point.X) * (point.Y + next.Y);
    }
    EXPECT_LT(area, 0) << "Outer polygon should be counter-clockwise.";

    const ConstPolygonRef inner = part[1];
    area = 0;
    for (size_t point_index = 0; point_index < inner.size(); point_index++)
    {
        const size_t next_index = (point_index + 1) % inner.size();
        const Point2LL point = inner[point_index];
        const Point2LL next = inner[next_index];
        area += (next.X - point.X) * (point.Y + next.Y);
    }
    EXPECT_GT(area, 0) << "Inner polygon should be clockwise.";
}

/*
 * The convex hull of a cube should still be a cube
 */
TEST_F(PolygonTest, convexTestCube)
{
    Polygons d_polygons;
    PolygonRef d = d_polygons.newPoly();
    d.add(Point2LL(0, 0));
    d.add(Point2LL(10, 0));
    d.add(Point2LL(10, 10));
    d.add(Point2LL(0, 10));

    d_polygons.makeConvex();

    EXPECT_EQ(d.size(), 4);
    EXPECT_EQ(d[0], Point2LL(0, 0));
    EXPECT_EQ(d[1], Point2LL(10, 0));
    EXPECT_EQ(d[2], Point2LL(10, 10));
    EXPECT_EQ(d[3], Point2LL(0, 10));
}

/*
 * The convex hull of a star should remove the inner points of the star
 */
TEST_F(PolygonTest, convexHullStar)
{
    Polygons d_polygons;
    PolygonRef d = d_polygons.newPoly();

    const int num_points = 10;
    const int outer_radius = 20;
    const int inner_radius = 10;
    const double angle_step = std::numbers::pi * 2.0 / num_points;
    for (int i = 0; i < num_points; ++i)
    {
        coord_t x_outer = -std::cos(angle_step * i) * outer_radius;
        coord_t y_outer = -std::sin(angle_step * i) * outer_radius;
        d.add(Point2LL(x_outer, y_outer));

        coord_t x_inner = -std::cos(angle_step * (i + 0.5)) * inner_radius;
        coord_t y_inner = -std::sin(angle_step * (i + 0.5)) * inner_radius;
        d.add(Point2LL(x_inner, y_inner));
    }

    d_polygons.makeConvex();

    EXPECT_EQ(d.size(), num_points);
    for (int i = 0; i < num_points; ++i)
    {
        double angle = angle_step * i;
        coord_t x = -std::cos(angle) * outer_radius;
        coord_t y = -std::sin(angle) * outer_radius;
        EXPECT_EQ(d[i], Point2LL(x, y));
    }
}

/*
 * Multiple min-x points
 * the convex hull the point with minimal x value. if there are multiple it might go wrong
 */
TEST_F(PolygonTest, convexHullMultipleMinX)
{
    Polygons d_polygons;
    PolygonRef d = d_polygons.newPoly();
    d.add(Point2LL(0, 0));
    d.add(Point2LL(0, -10));
    d.add(Point2LL(10, 0));
    d.add(Point2LL(0, 10));

    /*
     *   x\                          x\
     *   | \                        | \
     *   x  x    should result in   |  x
     *   | /                        | /
     *   x/                         x/
     *
     */

    d_polygons.makeConvex();

    EXPECT_EQ(d.size(), 3);
}

/*
 * The convex hull should remove collinear points
 */
TEST_F(PolygonTest, convexTestCubeColinear)
{
    Polygons d_polygons;
    PolygonRef d = d_polygons.newPoly();
    d.add(Point2LL(0, 0));
    d.add(Point2LL(5, 0));
    d.add(Point2LL(10, 0));
    d.add(Point2LL(10, 5));
    d.add(Point2LL(10, 10));
    d.add(Point2LL(5, 10));
    d.add(Point2LL(0, 10));
    d.add(Point2LL(0, 5));

    d_polygons.makeConvex();

    EXPECT_EQ(d.size(), 4);
    EXPECT_EQ(d[0], Point2LL(0, 0));
    EXPECT_EQ(d[1], Point2LL(10, 0));
    EXPECT_EQ(d[2], Point2LL(10, 10));
    EXPECT_EQ(d[3], Point2LL(0, 10));
}

/*
 * The convex hull should remove duplicate points
 */
TEST_F(PolygonTest, convexHullRemoveDuplicatePoints)
{
    Polygons d_polygons;
    PolygonRef d = d_polygons.newPoly();
    d.add(Point2LL(0, 0));
    d.add(Point2LL(0, 0));
    d.add(Point2LL(10, 0));
    d.add(Point2LL(10, 0));
    d.add(Point2LL(10, 10));
    d.add(Point2LL(10, 10));
    d.add(Point2LL(0, 10));
    d.add(Point2LL(0, 10));

    d_polygons.makeConvex();

    EXPECT_EQ(d.size(), 4);
    EXPECT_EQ(d[0], Point2LL(0, 0));
    EXPECT_EQ(d[1], Point2LL(10, 0));
    EXPECT_EQ(d[2], Point2LL(10, 10));
    EXPECT_EQ(d[3], Point2LL(0, 10));
}

/*
 * Check that a simple set of polygons do not change when run through
 * removeSmallAreas.
 */
TEST_F(PolygonTest, removeSmallAreas_simple)
{
    // basic set of polygons
    auto test_square_2 = test_square;
    test_square_2.translate(Point2LL(0, 500));
    auto d_polygons = Polygons{};
    d_polygons.add(test_square);
    d_polygons.add(test_square_2);
    d_polygons.add(triangle);

    // for the simple case there should be no change.
    auto act_polygons = d_polygons;
    act_polygons.removeSmallAreas(1e-5, false);
    twoPolygonsAreEqual(act_polygons, d_polygons);

    // changing remove_holes should have no effect.
    act_polygons = d_polygons;
    act_polygons.removeSmallAreas(1e-5, true);
    twoPolygonsAreEqual(act_polygons, d_polygons);
}

/*
 * Check that the two small areas are removed but the two large areas are not
 * affected.
 */
TEST_F(PolygonTest, removeSmallAreas_small_area)
{
    // make some areas.
    auto small_area_1 = small_area; // Area = 100 micron^2 = 1e-4 mm^2
    small_area_1.translate(Point2LL(350, 450));
    auto small_area_2 = small_area;
    small_area_2.translate(Point2LL(450, 350));
    auto triangle_1 = triangle; // area = 10000 micron^2 = 1e-2 mm^2
    triangle_1.translate(Point2LL(50, 0));

    // add areas to polygons
    auto d_polygons = Polygons{};
    d_polygons.add(small_area_1);
    d_polygons.add(small_area_2);
    d_polygons.add(test_square); // area = 10000 micron^2 = 1e-2 mm^2
    d_polygons.add(triangle_1);

    // make an expected Polygons
    auto exp_polygons = Polygons{};
    exp_polygons.add(test_square);
    exp_polygons.add(triangle_1);

    // for remove_holes == false, 2 poly removed
    auto act_polygons = d_polygons;
    act_polygons.removeSmallAreas(1e-3, false);
    twoPolygonsAreEqual(act_polygons, exp_polygons);

    // for remove_holes == true, 2 poly removed
    act_polygons = d_polygons;
    act_polygons.removeSmallAreas(1e-3, true);
    twoPolygonsAreEqual(act_polygons, exp_polygons);
}

/*
 * Check that that a small hole in a large area is only removed if the setting
 * is true.
 */
TEST_F(PolygonTest, removeSmallAreas_hole)
{
    // make some areas.
    auto small_hole_1 = small_area; // Area = 100 micron^2 = 1e-4 mm^2
    small_hole_1.reverse();
    small_hole_1.translate(Point2LL(10, 10));

    // add areas to polygons
    auto d_polygons = Polygons{};
    d_polygons.add(test_square); // area = 10000 micron^2 = 1e-2 mm^2
    d_polygons.add(small_hole_1);


    // for remove_holes == false there should be no change.
    auto act_polygons = d_polygons;
    act_polygons.removeSmallAreas(1e-3, false);
    twoPolygonsAreEqual(act_polygons, d_polygons);

    // for remove_holes == true there should be one less poly.
    // make an expected Polygons
    auto exp_polygons = Polygons{};
    exp_polygons.add(test_square);
    act_polygons = d_polygons;
    act_polygons.removeSmallAreas(1e-3, true);
    twoPolygonsAreEqual(act_polygons, exp_polygons);
}

/*
 * Test that a hole inside a removed area is always removed.
 */
TEST_F(PolygonTest, removeSmallAreas_hole_2)
{
    // make some areas.
    auto small_hole_1 = small_area; // Area = 100 micron^2 = 1e-4 mm^2
    small_hole_1.reverse();
    auto small_hole_2 = small_hole_1;
    small_hole_1.translate(Point2LL(10, 10));
    small_hole_2.translate(Point2LL(160, 160));
    auto med_square_1 = Polygon{}; // area = 2500 micron^2 = 2.5e-3 mm^2
    med_square_1.add(Point2LL(0, 0));
    med_square_1.add(Point2LL(50, 0));
    med_square_1.add(Point2LL(50, 50));
    med_square_1.add(Point2LL(0, 50));
    med_square_1.translate(Point2LL(150, 150));

    // add areas to polygons
    auto d_polygons = Polygons{};
    d_polygons.add(test_square); // area = 10000 micron^2 = 1e-2 mm^2
    d_polygons.add(small_hole_1);
    d_polygons.add(med_square_1);
    d_polygons.add(small_hole_2);

    // for remove_holes == false, two polygons removed.
    auto act_polygons = d_polygons;
    // make an expected Polygons
    auto exp_polygons = Polygons{};
    exp_polygons.add(test_square);
    exp_polygons.add(small_hole_1);
    act_polygons.removeSmallAreas(3e-3, false);
    twoPolygonsAreEqual(act_polygons, exp_polygons);

    // for remove_holes == true, three polygons removed.
    act_polygons = d_polygons;
    // make an expected Polygons
    exp_polygons = Polygons{};
    exp_polygons.add(test_square);
    act_polygons.removeSmallAreas(3e-3, true);
    twoPolygonsAreEqual(act_polygons, exp_polygons);
}

/*
 * Test the following:
 *   1. Two large areas (triangle and square) are not removed.
 *   2. Two small holes in the square are removed if remove_holes==true.
 *   3. Three small areas are always removed.
 */
TEST_F(PolygonTest, removeSmallAreas_complex)
{
    // make some areas.
    auto small_area_1 = small_area; // Area = 100 micron^2 = 1e-4 mm^2
    small_area_1.translate(Point2LL(350, 450));
    auto small_area_2 = small_area;
    small_area_2.translate(Point2LL(450, 350));
    auto small_hole_1 = small_area; // Area = 100 micron^2 = 1e-4 mm^2
    small_hole_1.reverse();
    auto small_hole_2 = small_hole_1;
    small_hole_1.translate(Point2LL(3, 3));
    small_hole_2.translate(Point2LL(22, 50));
    auto triangle_1 = triangle; // area = 10000 micron^2 = 1e-2 mm^2
    triangle_1.translate(Point2LL(600, 0));

    // add areas to polygons
    auto d_polygons = Polygons{};
    d_polygons.add(small_area_1);
    d_polygons.add(small_area_2);
    d_polygons.add(test_square); // area = 10000 micron^2 = 1e-2 mm^2
    d_polygons.add(small_hole_1);
    d_polygons.add(small_hole_2);
    d_polygons.add(triangle_1);

    // for remove_holes == false there should be 2 small areas removed.
    auto act_polygons = d_polygons;
    // make an expected Polygons
    auto exp_polygons = Polygons{};
    exp_polygons.add(test_square); // area = 10000 micron^2 = 1e-2 mm^2
    exp_polygons.add(small_hole_1);
    exp_polygons.add(small_hole_2);
    exp_polygons.add(triangle_1);
    act_polygons.removeSmallAreas(1e-3, false);
    twoPolygonsAreEqual(act_polygons, exp_polygons);

    // for remove_holes == true there should be 2 small areas and 2 small holes removed.
    act_polygons = d_polygons;
    // make an expected Polygons
    exp_polygons = Polygons{};
    exp_polygons.add(test_square);
    exp_polygons.add(triangle_1); // area = 10000 micron^2 = 1e-2 mm^2
    act_polygons.removeSmallAreas(1e-3, true);
    twoPolygonsAreEqual(act_polygons, exp_polygons);
}
} // namespace cura
// NOLINTEND(*-magic-numbers)
