//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "AABBTest.h"

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(AABBTest);

void AABBTest::setUp()
{
    test_square.emplace_back(0, 0);
    test_square.emplace_back(100, 0);
    test_square.emplace_back(100, 100);
    test_square.emplace_back(0, 100);

    test_triangle.emplace_back(0, 0);
    test_triangle.emplace_back(-50, 40);
    test_triangle.emplace_back(60, 60);

    Point point_min = Point(-10, -12);
    Point point_max = Point(11, 13);
    my_aabb = AABB(point_min, point_max);

}

void AABBTest::tearDown()
{
    //Do nothing.
}


void AABBTest::smokeTest()
{
    Point point_min = Point(-10, -12);
    Point point_max = Point(11, 13);
    AABB aabb(point_min, point_max);
}

void AABBTest::smokeTest2()
{
    Polygons polys = Polygons();
    polys.add(test_square);
    AABB aabb(polys);
}

void AABBTest::smokeTest3()
{
    Polygons test_polys;
    PolygonRef poly = test_polys.newPoly();
    poly.add(Point(82124,98235));
    poly.add(Point(83179,98691));
    poly.add(Point(83434,98950));
    poly.add(Point(82751,99026));
    poly.add(Point(82528,99019));
    ConstPolygonRef cpoly = ConstPolygonRef(poly);
    AABB aabb(cpoly);
}

void AABBTest::calculateTest()
{
    Polygons polys = Polygons();
    polys.add(test_triangle);
    my_aabb.calculate(polys);
}

void AABBTest::getMiddleTest()
{

}

void AABBTest::hitTest()
{

}

void AABBTest::includeTest()
{

}

void AABBTest::expandTest()
{

}


}
