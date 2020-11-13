//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <../src/utils/polygon.h> //The class under test.
#include <../src/utils/polygonUtils.h> // helper functions
#include <../src/utils/SVG.h> // helper functions

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
    
    static constexpr bool visualize = false;

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

    for (size_t point_index = 1; point_index + 1 < circle.size(); point_index++) //Don't check the last vertex. Due to odd-numbered vertices it has to be shorter than the minimum.
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
    size_t zigzag_count = 100;

    for (size_t i = 0; i < zigzag_count; i++)
    {
        zigzag.add(Point(x, i * y_increment));
        x = 0 - x;
    }

    Polygon zigzag_before = zigzag;

    // we make the smallest_line_segment just smaller than the line from the start to the second to last point
    // so that we ensure the polygon does not get removed altogether
    zigzag_polygons.simplify(y_increment * (zigzag_count - 2) - 100, y_increment);

    if (visualize)
    {
        SVG svg("output/simplifyZigzag.svg", AABB(zigzag_before));
        svg.writePolygon(zigzag_before);
        svg.nextLayer();
        svg.writePolygon(zigzag, SVG::Color::RED);
    }
    
    EXPECT_THAT(zigzag.size(), testing::AllOf(testing::Ge(3), testing::Le(5))) << "All but the last zigzag should be removed.";
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
    // Generate a large polygon with a small region with arbitrary simplifiable geometry
    // We choose a spiral for that arbitrary geometry
    
    //Generate a square spiral with increasingly large corners
    Polygons spiral_polygons;
    PolygonRef spiral = spiral_polygons.newPoly();
    spiral.emplace_back(-15000, 11000); // introduce points that may never be simplified so that we know the simplification should start at 0,0
    spiral.emplace_back(-15000, 1000);
    spiral.emplace_back(0, 0);

    //Generate a square spiral, 90 degree corners to make it easy to compute the area loss while retaining a positive area per corner.
    coord_t segment_length = 1000;
    Point last_position(0, 0);
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
    Polygon spiral_before = spiral;

    coord_t max_height = segment_length * std::sqrt(2.0); // the diameter of the circle along the diagonal

    for (size_t i = 0; i < spiral_before.size(); ++i)
    {
        // apply simplify iteratively for each point until nothing is simplifiable any more
        spiral_polygons.simplify(10000, max_height);
    }
    
    if (visualize)
    {
        SVG svg("output/simplifyLimitedError.svg", AABB(spiral_before));
        svg.writePolygon(spiral_before);
        svg.nextLayer();
        svg.writePolygon(spiral, SVG::Color::RED);
    }

    EXPECT_THAT(spiral.size(), testing::Eq(4)) << "Should simplify all spiral points except those connected to far away geometry.";
}


TEST_F(PolygonTest, simplifyIncreasingLimitedError)
{
    // Generate a zigzag with bends in the outer ends with increasing height
    //   _   /\                        .
    //  | | | |
    //  | | | |
    //  | | | |
    //  | | | |
    //     v   \/
    
    Polygons zigzag_polygons;
    PolygonRef zigzag = zigzag_polygons.newPoly();
    
    constexpr coord_t non_simplifiable_bound = 10000;
    constexpr coord_t max_height = 562;
    
    coord_t long_segment = non_simplifiable_bound + 100;
    
    constexpr coord_t width = 1000;
    
    zigzag.emplace_back(0, 0);

    size_t simplifiable_bend_count = 0;
    
    for (coord_t height = 100; height < 1000 || (height == 1000 && long_segment < 0); height += 25)
    {
        Point last_position = zigzag.back();
        zigzag.emplace_back(last_position + Point(width / 2, height));
        zigzag.emplace_back(last_position + Point(width, 0));
        zigzag.emplace_back(last_position + Point(width, long_segment));
        long_segment *= -1;
        if (height < max_height)
        {
            simplifiable_bend_count++;
        }
    }
    zigzag.emplace_back(zigzag.back() + Point(-non_simplifiable_bound, -non_simplifiable_bound)); // complete polygon with a point which enforces non-simplification to both its segments

    Polygon zigzag_before = zigzag;

    zigzag_polygons.simplify(non_simplifiable_bound, max_height);

    if (visualize)
    {
        SVG svg("output/simplifyIncreasingLimitedError.svg", AABB(zigzag_before));
        svg.writePolygon(zigzag_before);
        svg.nextLayer();
        svg.writePolygon(zigzag, SVG::Color::RED);
    }
    EXPECT_THAT(zigzag.size(), testing::Eq(zigzag_before.size() - simplifiable_bend_count)) << "Should simplify bends with height 100 up to 500";
}


TEST_F(PolygonTest, simplifySineLimitedError)
{
    // Generate a straight line with sinusoidal errors which should be simplified back into a more straight line
    
    // Hypothetically simplify() might replace each half period of the sine with a straight segment,
    // but because simplify() is heuristic it introduces more segments.
    // The function signature doesn't provide any guarantee about how much simplification will occur,
    // but in practice it should at least simplify up to double the minimal segment count.
    
    Polygons sine_polygons;
    PolygonRef sine = sine_polygons.newPoly();
    
    constexpr coord_t length = 10000;
    constexpr coord_t deviation = 500;
    constexpr size_t bulge_count = 5;
    
    sine.emplace_back(length, 0);
    sine.emplace_back(length, length);
    sine.emplace_back(0, length);
    sine.emplace_back(0, 0);

    for (coord_t x = 100; x < length; x += 100)
    {
        sine.emplace_back(x, std::sin(INT2MM(x) / INT2MM(length) * M_PI * bulge_count ) * deviation);
    }
    Polygon sine_before = sine;

    sine_polygons.simplify(length / 2, 2 * deviation);
    if (visualize)
    {
        SVG svg("output/simplifySineLimitedError.svg", AABB(sine_before));
        svg.writePolygon(sine_before);
        svg.nextLayer();
        svg.writePolygon(sine, SVG::Color::RED);
    }
    const size_t max_simplified_sine_segments = bulge_count * 2; // * 2 because simplify() is not precise and might not optimally simplify
    EXPECT_THAT(sine.size(), testing::AllOf(testing::Ge(4), testing::Le(4 + max_simplified_sine_segments))) << "Should simplify each outward and each inward bulge.";
}

TEST_F(PolygonTest, simplifySineHighPoly)
{
    // Generate a straight line with sinusoidal errors which should be simplified back into a more straight line
    
    // Hypothetically simplify() might replace each half period of the sine with a straight segment,
    // but because simplify() is heuristic it introduces more segments.
    // The function signature doesn't provide any guarantee about how much simplification will occur,
    // but in practice it should at least simplify up to double the minimal segment count.
    
    Polygons sine_polygons;
    PolygonRef sine = sine_polygons.newPoly();
    
    constexpr coord_t length = 1000;
    constexpr coord_t deviation = 100;
    constexpr size_t bulge_count = 17;
    constexpr coord_t allowed_simplification_height = 30;
    constexpr coord_t allowed_segment_length = 100;
    
    sine.emplace_back(length, 0);
    sine.emplace_back(length, length);
    sine.emplace_back(0, length);
    sine.emplace_back(0, 0);

    for (coord_t x = 1; x < length; x += 1)
    {
        coord_t y = std::sin(INT2MM(x) / INT2MM(length) * M_PI * bulge_count) * deviation;
        if ( ! sine.empty())
        {
            coord_t y_mid = (y + sine.back().Y) / 2;
            if (y_mid != y && y_mid != sine.back().Y
                && sine.back().X == x - 1
            )
            {
                sine.emplace_back(x, y_mid);
            }
        }
        sine.emplace_back(x, y);
        if (x % (allowed_segment_length * 3) == 0) x += allowed_segment_length + 5;
    }
    Polygon sine_before = sine;

    sine_polygons.simplify(allowed_segment_length, allowed_simplification_height);
    
    // find largest height deviation
    coord_t largest_dist = 0;
    for (Point from : sine_before)
    {
        ClosestPolygonPoint cpp = PolygonUtils::findClosest(from, sine_polygons);
        coord_t dist_between_polys = vSize(from - cpp.p());
        largest_dist = std::max(largest_dist, dist_between_polys);
    }
    if (visualize)
    {
        SVG svg("output/simplifySineHighPoly.svg", AABB(sine_before));
        svg.writePolygon(sine_before);
        svg.nextLayer();
        svg.writePolygon(sine, SVG::Color::RED);
    }
    EXPECT_THAT( largest_dist, testing::Le(allowed_simplification_height + 10)) << "Shouldn't exceed maximum error distance";
}

TEST_F(PolygonTest, simplifyCircleLimitedError)
{
    //Generate a circle with increasing resolution
    Polygons circle_polygons;
    PolygonRef circle = circle_polygons.newPoly();

    coord_t radius = 20000;
    coord_t segment_length = 100;
    for (double angle = 0; angle < 2 * M_PI; )
    {
        const coord_t dx = std::cos(angle) * radius;
        const coord_t dy = std::sin(angle) * radius;
        Point new_point(dx, dy);
        assert(circle.empty() || std::abs( vSize(circle.back() - new_point) - segment_length) < 10); // we should now add a segment of the prescribed length
        circle.add(new_point);
        segment_length += 100;
        angle += 2.0 * std::asin(0.5 * INT2MM(segment_length) / INT2MM(radius));
    }

    Polygon circle_before = circle;

    coord_t allowed_simplification_height = 1000;
    Polygons circle_polygons_before = circle_polygons;
    
    circle_polygons.simplify(9999999, allowed_simplification_height);

    // find largest height deviation
    coord_t largest_dist = 0;
    for (Point from : circle_before)
    {
        ClosestPolygonPoint cpp = PolygonUtils::findClosest(from, circle_polygons);
        coord_t dist_between_polys = vSize(from - cpp.p());
        largest_dist = std::max(largest_dist, dist_between_polys);
    }
    if (visualize)
    {
        SVG svg("output/simplifyCircleLimitedError.svg", AABB(circle_before));
        svg.writePolygon(circle_before);
        svg.nextLayer();
        svg.writePolygon(circle, SVG::Color::RED);
    }
    EXPECT_THAT( largest_dist, testing::Le(allowed_simplification_height + 10)) << "Shouldn't exceed maximum error distance";
}

TEST_F(PolygonTest, simplifyCircleHighPoly)
{
    //Generate a circle with extremely high point count, such that all segments are within rounding distance 
    Polygons circle_polygons;
    PolygonRef circle = circle_polygons.newPoly();

    coord_t radius = 2000;
    coord_t segment_length = 1;
    coord_t allowed_simplification_height = 50;

    for (double angle = 0; angle < 2 * M_PI; )
    {
        const coord_t x = std::cos(angle) * radius;
        const coord_t y = std::sin(angle) * radius;
        Point new_point(x, y);
        circle.add(new_point);
        coord_t segment_length_here = segment_length;
        if ( (x + y) % 5 == 0 )
        {
            segment_length_here = 500;
        }
        angle += 2.0 * std::asin(0.5 * INT2MM(segment_length_here) / INT2MM(radius));
    }

    Polygon circle_before = circle;

    Polygons circle_polygons_before = circle_polygons;
    
    circle_polygons.simplify(9999999, allowed_simplification_height);

    // find largest height deviation
    coord_t largest_dist = 0;
    for (Point from : circle_before)
    {
        ClosestPolygonPoint cpp = PolygonUtils::findClosest(from, circle_polygons);
        coord_t dist_between_polys = vSize(from - cpp.p());
        largest_dist = std::max(largest_dist, dist_between_polys);
    }
    if (visualize)
    {
        SVG svg("output/simplifyCircleHighPoly.svg", AABB(circle_before));
        svg.writePolygon(circle_before);
        svg.nextLayer();
        svg.writePolygon(circle, SVG::Color::RED);
    }
    EXPECT_THAT( largest_dist, testing::Le(allowed_simplification_height + 5)) << "Shouldn't exceed maximum error distance";
}

TEST_F(PolygonTest, simplifyColinear)
{
    //Generate a line with several vertices halfway.
    constexpr coord_t spacing = 100;
    Polygons colinear_polygons;
    PolygonRef colinear = colinear_polygons.newPoly();
    for(size_t i = 0; i < 10; i++)
    {
        colinear.add(Point(i * spacing + i % 2 - 1, i * spacing + i % 2 - 1)); //Some jitter of 2 microns is allowed.
    }
    colinear.add(Point(spacing * 9, 0)); //Make it a triangle so that the area is not 0 or anything.

    Polygon colinear_before = colinear;

    colinear_polygons.simplify(20, 20); //Regardless of parameters, it should always remove vertices with less than 5 micron deviation.
    if (visualize)
    {
        SVG svg("output/simplifyColinear.svg", AABB(colinear_before));
        svg.writePolygon(colinear_before);
        svg.nextLayer();
        svg.writePolygon(colinear, SVG::Color::RED);
    }
    ASSERT_EQ(colinear_polygons.size(), 1) << "Polygon shouldn't have gotten removed altogether";
    ASSERT_LE(colinear_polygons[0].size(), 8) << "At least half of the colinear points should have been removed";
}

TEST_F(PolygonTest, simplifyDegenerateVertex)
{
    //Generate a line with several vertices halfway.
    constexpr coord_t spacing = 100;
    Polygons colinear_polygons;
    PolygonRef colinear = colinear_polygons.newPoly();
    colinear.emplace_back(0, 0);
    colinear.emplace_back(spacing, 0);
    colinear.emplace_back(spacing, spacing);
    colinear.emplace_back(0, spacing);
    colinear.emplace_back(0, -spacing); // degenerate vertex
    
    Polygon colinear_before = colinear;

    colinear_polygons.simplify(20, 5); //Regardless of parameters, it should always remove the one vertex
    if (visualize)
    {
        SVG svg("output/simplifyDegenerateVertex.svg", AABB(colinear_before));
        svg.writePolygon(colinear_before);
        svg.nextLayer();
        svg.writePolygon(colinear, SVG::Color::RED);
    }
    ASSERT_EQ(colinear_polygons.size(), 1) << "Polygon shouldn't have gotten removed altogether";
    ASSERT_EQ(colinear_polygons[0].size(), 4) << "The one colinear vertex should have gotten removed.";
}

/*
 * Test whether a polygon can be reduced to 1 or 2 vertices. In that case, it
 * should get reduced to 0 or stay at 3.
 */
TEST_F(PolygonTest, simplifyToDegenerate)
{
    //Generate a D shape with one long side and another curved side broken up into smaller pieces that can be removed.
    Polygons d_polygons;
    PolygonRef d = d_polygons.newPoly();
    d.add(Point(0, 0));
    d.add(Point(10, 55));
    d.add(Point(0, 110));

    d.simplify(100, 15);

    EXPECT_NE(d.size(), 1);
    EXPECT_NE(d.size(), 2);
}

}
