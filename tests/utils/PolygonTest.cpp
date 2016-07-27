//Copyright (c) 2015 Ultimaker B.V.
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




}