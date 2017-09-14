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

void PolygonTest::clockwiseTest()
{
    PolygonsPart part = clockwise_donut.splitIntoParts()[0];
    {
        PolygonRef outer_after = part.outerPolygon();
        unsigned int start_idx;
        for (start_idx = 0; start_idx < outer_after.size(); start_idx++)
        {
            if (outer_after[start_idx] == Point(-100, -100))
            {
                break;
            }
        }
        CPPUNIT_ASSERT_MESSAGE("Outer polygon is not counter-clockwise!", outer_after[(start_idx + 1) % outer_after.size()] == Point(100, -100));
    }
    {
        PolygonRef inner_after = part[1];
        unsigned int start_idx;
        for (start_idx = 0; start_idx < inner_after.size(); start_idx++)
        {
            if (inner_after[start_idx] == Point(-50, -50))
            {
                break;
            }
        }
        CPPUNIT_ASSERT_MESSAGE("Hole polygon is not clockwise!", inner_after[(start_idx + 1) % inner_after.size()] == Point(-50, 50));
    }
}


}
