//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "AABBTest.h"
#include <iomanip>

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(AABBTest);

void AABBTest::setUp()
{
    test_rect.emplace_back(0, 0);
    test_rect.emplace_back(100, 0);
    test_rect.emplace_back(100, 110);
    test_rect.emplace_back(0, 110);

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
    polys.add(test_rect);
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
    std::stringstream ss;
    Polygons polys = Polygons();
    polys.add(test_triangle);
    my_aabb.calculate(polys);
    ss << "Min X was not expected: " << my_aabb.min.X << ".";
    CPPUNIT_ASSERT_MESSAGE(ss.str(), std::abs(my_aabb.min.X - -50) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Max X was not expected", std::abs(my_aabb.max.X - 60) < epsilon);
    ss << "Min Y was not expected: " << my_aabb.min.Y << ".";
    CPPUNIT_ASSERT_MESSAGE(ss.str(), std::abs(my_aabb.min.Y - 0) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Max Y was not expected", std::abs(my_aabb.max.Y - 60) < epsilon);
}

void AABBTest::getMiddleTest()
{
    std::stringstream ss;
    Point result = my_aabb.getMiddle();
    ss << "Middle X value is not expected: " << std::fixed << std::setw( 11 ) << std::setprecision( 6 )
          << std::setfill( '0' ) << result.X << ".";
    CPPUNIT_ASSERT_MESSAGE(ss.str(), std::abs(result.X - 0) < epsilon);  // Apparently the values are rounded down
    CPPUNIT_ASSERT_MESSAGE("Middle Y value is not expected.", std::abs(result.Y - 0) < epsilon);  // Apparently the values are rounded down
}

void AABBTest::hitTest()
{
    Point point_min = Point(12, 0);
    Point point_max = Point(130, 30);
    AABB aabb_hit = AABB(point_min, point_max);
    CPPUNIT_ASSERT_MESSAGE("AABB should not have hit", my_aabb.hit(aabb_hit) == false);
}

void AABBTest::hitTest2()
{
    Point point_min = Point(10, 12);
    Point point_max = Point(15, 30);
    AABB aabb_hit = AABB(point_min, point_max);
    CPPUNIT_ASSERT_MESSAGE("AABB should have hit", my_aabb.hit(aabb_hit) == true);
}

void AABBTest::includeTest()
{
    std::stringstream ss;
    Point point_include = Point(100, -20);
    my_aabb.include(point_include);
    ss << "Min X was not expected: " << my_aabb.min.X << ".";
    CPPUNIT_ASSERT_MESSAGE(ss.str(), std::abs(my_aabb.min.X - -10) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Max X was not expected", std::abs(my_aabb.max.X - 100) < epsilon);
    ss << "Min Y was not expected: " << my_aabb.min.Y << ".";
    CPPUNIT_ASSERT_MESSAGE(ss.str(), std::abs(my_aabb.min.Y - -20) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Max Y was not expected", std::abs(my_aabb.max.Y - 13) < epsilon);
}

void AABBTest::expandTest()
{
    my_aabb.expand(5);
    CPPUNIT_ASSERT_MESSAGE("Min X was not expected", std::abs(my_aabb.min.X - -15) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Max X was not expected", std::abs(my_aabb.max.X - 16) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Min Y was not expected", std::abs(my_aabb.min.Y - -17) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Max Y was not expected", std::abs(my_aabb.max.Y - 18) < epsilon);
}


}
