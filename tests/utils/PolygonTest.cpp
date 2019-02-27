//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PolygonTest.h"

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(PolygonTest);

void PolygonTest::setUp()
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

void PolygonTest::tearDown()
{
    //Do nothing.
}


void PolygonTest::polygonOffsetTest()
{
    Polygons test_squares;
    test_squares.add(test_square);
    Polygons expanded = test_squares.offset(25);
    int64_t expanded_length = expanded.polygonLength();

    Polygons square_hole;
    PolygonRef square_inverted = square_hole.newPoly();
    for (int i = test_square.size() - 1; i >= 0; i--)
    {
        square_inverted.add(test_square[i]);
    }
    Polygons contracted = square_hole.offset(25);
    int64_t contracted_length = contracted.polygonLength();

    CPPUNIT_ASSERT_MESSAGE("Offset on outside poly is different from offset on inverted poly!", std::abs(expanded_length - contracted_length) < 5);
}

void PolygonTest::polygonOffsetBugTest()
{
    Polygons polys;
    polys.add(clipper_bug);
    Polygons offsetted = polys.offset(-20);

    for (PolygonRef poly : offsetted)
    {
        for (Point& p : poly)
        {
            CPPUNIT_ASSERT_MESSAGE("Polygon offset moved point the wrong way!", polys.inside(p));
        }
    }
}


void PolygonTest::isOutsideTest()
{
    Polygons test_triangle;
    test_triangle.add(triangle);

    CPPUNIT_ASSERT_MESSAGE("Left point is calculated as inside while it's outside!", !test_triangle.inside(Point(0, 100)));
    CPPUNIT_ASSERT_MESSAGE("Middle left point is calculated as inside while it's outside!", !test_triangle.inside(Point(100, 100)));
    CPPUNIT_ASSERT_MESSAGE("Middle right point is calculated as inside while it's outside!", !test_triangle.inside(Point(300, 100)));
    CPPUNIT_ASSERT_MESSAGE("Right point is calculated as inside while it's outside!", !test_triangle.inside(Point(500, 100)));
    CPPUNIT_ASSERT_MESSAGE("Above point is calculated as inside while it's outside!", !test_triangle.inside(Point(100, 200)));
    CPPUNIT_ASSERT_MESSAGE("Below point is calculated as inside while it's outside!", !test_triangle.inside(Point(100, -100)));
}

void PolygonTest::isInsideTest()
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

    CPPUNIT_ASSERT_MESSAGE("Inside point is calculated as being outside!", test_polys.inside(Point(78315, 98440)));

}

void PolygonTest::splitIntoPartsWithHoleTest()
{
    const std::vector<PolygonsPart> parts = clockwise_donut.splitIntoParts();

    CPPUNIT_ASSERT_MESSAGE("difference between two polygons is not one PolygonsPart!", parts.size() == 1);
}

void PolygonTest::differenceContainsOriginalPointTest()
{
    PolygonsPart part = clockwise_donut.splitIntoParts()[0];
    PolygonRef outer = part.outerPolygon();
    CPPUNIT_ASSERT_MESSAGE("Outer vertex cannot be found in polygons difference!", std::find(outer.begin(), outer.end(), clockwise_large[0]) != outer.end());
    PolygonRef inner = part[1];
    CPPUNIT_ASSERT_MESSAGE("Inner vertex cannot be found in polygons difference!", std::find(inner.begin(), inner.end(), clockwise_small[0]) != inner.end());
}

void PolygonTest::differenceClockwiseTest()
{
    PolygonsPart part = clockwise_donut.splitIntoParts()[0];

    PolygonRef outer = part.outerPolygon();
    //Apply the shoelace formula to determine surface area. If it's negative, the polygon is counterclockwise.
    coord_t area = 0;
    for (size_t point_index = 0; point_index < outer.size(); point_index++)
    {
        const size_t next_index = (point_index + 1) % outer.size();
        const Point point = outer[point_index];
        const Point next = outer[next_index];
        area += (next.X - point.X) * (point.Y + next.Y);
    }
    CPPUNIT_ASSERT_MESSAGE("Outer polygon is not counter-clockwise!", area < 0);

    PolygonRef inner = part[1];
    area = 0;
    for (size_t point_index = 0; point_index < inner.size(); point_index++)
    {
        const size_t next_index = (point_index + 1) % inner.size();
        const Point point = inner[point_index];
        const Point next = inner[next_index];
        area += (next.X - point.X) * (point.Y + next.Y);
    }
    CPPUNIT_ASSERT_MESSAGE("Inner polygon is not clockwise!", area > 0);
}

void PolygonTest::getEmptyHolesTest()
{
    Polygons holes = clockwise_donut.getEmptyHoles();

    CPPUNIT_ASSERT_MESSAGE("Not the correct number of holes!", holes.size() == 1);
    CPPUNIT_ASSERT_MESSAGE("Empty hole doesn't have the correct amount of vertices!", holes[0].size() == clockwise_small.size());
    for (size_t point_index = 0; point_index < holes[0].size(); point_index++)
    {
        CPPUNIT_ASSERT_MESSAGE("Coordinates of empty hole are wrong!", holes[0][point_index] == clockwise_small[point_index]);
    }
}

void PolygonTest::simplifyCircle()
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
        std::stringstream ss_short;
        ss_short << "Segment " << (point_index - 1) << " - " << point_index << " is too short! " << segment_length << " < " << minimum_segment_length;
        CPPUNIT_ASSERT_MESSAGE(ss_short.str(), segment_length >= minimum_segment_length);
        std::stringstream ss_long;
        ss_long << "Segment " << (point_index - 1) << " - " << point_index << " is too long! " << segment_length << " > " << maximum_segment_length;
        CPPUNIT_ASSERT_MESSAGE(ss_long.str(), segment_length <= maximum_segment_length);
    }
}

void PolygonTest::simplifyZigzag()
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

    std::stringstream ss;
    ss << "Zigzag should be removed since the total error compensates with each zag, but size was " << zigzag.size() << ".";
    CPPUNIT_ASSERT_MESSAGE(ss.str(), zigzag.size() <= 5);
}

void PolygonTest::simplifyLimitedLength()
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

    CPPUNIT_ASSERT_MESSAGE(std::string("Should merge segments of length 1100 with 1200, 1300 with 1400 and first with last."), spiral.size() == 11 - 3);
}

void PolygonTest::simplifyLimitedError()
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

    CPPUNIT_ASSERT_MESSAGE(std::string("Should merge segments of length 1000 through 1400 and first with last."), spiral.size() == 11 - 5);
}

}
