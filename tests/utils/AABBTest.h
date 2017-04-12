//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef AABB_TEST_H
#define AABB_TEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <../src/utils/intpoint.h>
#include <../src/utils/AABB.h>
#include <../src/utils/polygon.h>
#include <clipper/clipper.hpp>

namespace cura
{

class AABBTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(AABBTest);
    CPPUNIT_TEST(smokeTest);
    CPPUNIT_TEST(smokeTest2);
    CPPUNIT_TEST(smokeTest3);
    CPPUNIT_TEST(calculateTest);
    CPPUNIT_TEST(getMiddleTest);
    CPPUNIT_TEST(hitTest);
    CPPUNIT_TEST(includeTest);
    CPPUNIT_TEST(expandTest);
    CPPUNIT_TEST_SUITE_END();

public:
    void setUp();

    void tearDown();

    void smokeTest();
    void smokeTest2();
    void smokeTest3();

    void calculateTest();
    void getMiddleTest();
    void hitTest();
    void includeTest();
    void expandTest();

private:
    Polygon test_square;
    Polygon test_triangle;
    AABB my_aabb;
};

}

#endif //AABB_TEST_H

