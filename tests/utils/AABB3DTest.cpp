//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.
//Use this file as a template for testing new classes
#include "AABB3DTest.h"

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(TemplateTest);

void TemplateTest::setUp()
{
    my_aabb3d = AABB3D();
    my_aabb3d.include(Point3(13,-11,-12));
    my_aabb3d.include(Point3(-10,14,15));
}

void TemplateTest::tearDown()
{
}


void TemplateTest::smokeTest()
{
    my_aabb3d.getMiddle();
}

void TemplateTest::hitTest()
{
    AABB3D other_aabb3d = AABB3D();
    other_aabb3d.include(Point3(-1,-1,-1));
    other_aabb3d.include(Point3(1,1,1));
    CPPUNIT_ASSERT_MESSAGE("Expected hit", my_aabb3d.hit(other_aabb3d) == true);
}

void TemplateTest::hitTest2()
{
    AABB3D other_aabb3d = AABB3D();
    other_aabb3d.include(Point3(-1,-1,16));
    other_aabb3d.include(Point3(1,1,18));
    CPPUNIT_ASSERT_MESSAGE("Not expected hit", my_aabb3d.hit(other_aabb3d) == false);
}

void TemplateTest::hitTest3()
{
    AABB3D other_aabb3d = AABB3D();
    other_aabb3d.include(Point3(12,-1,1));
    other_aabb3d.include(Point3(15,1,3));
    CPPUNIT_ASSERT_MESSAGE("Expected hit", my_aabb3d.hit(other_aabb3d) == true);
}

void TemplateTest::hitTest4()
{
    AABB3D other_aabb3d = AABB3D();
    other_aabb3d.include(Point3(14,-1,1));
    other_aabb3d.include(Point3(17,1,3));
    CPPUNIT_ASSERT_MESSAGE("Not expected hit", my_aabb3d.hit(other_aabb3d) == false);
}

void TemplateTest::includeTest()
{
    CPPUNIT_ASSERT_MESSAGE("Unexpected min x", std::abs(my_aabb3d.min.x - -10) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected min y", std::abs(my_aabb3d.min.y - -11) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected min z", std::abs(my_aabb3d.min.z - -12) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected max x", std::abs(my_aabb3d.max.x - 13) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected max y", std::abs(my_aabb3d.max.y - 14) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected max z", std::abs(my_aabb3d.max.z - 15) < epsilon);
}

void TemplateTest::includeZTest()
{
    my_aabb3d.includeZ(42);
    CPPUNIT_ASSERT_MESSAGE("Unexpected max z", std::abs(my_aabb3d.max.z - 42) < epsilon);
}

void TemplateTest::offsetTest()
{
    Point3 offset = Point3(15, 20, 30);
    my_aabb3d.offset(offset);
    CPPUNIT_ASSERT_MESSAGE("Unexpected min x", std::abs(my_aabb3d.min.x - -10-15) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected min y", std::abs(my_aabb3d.min.y - -11-20) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected min z", std::abs(my_aabb3d.min.z - -12-30) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected max x", std::abs(my_aabb3d.max.x - 13-15) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected max y", std::abs(my_aabb3d.max.y - 14-20) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected max z", std::abs(my_aabb3d.max.z - 15-30) < epsilon);
}

void TemplateTest::offsetTest2()
{
    Point offset = Point(15, 20);
    my_aabb3d.offset(offset);
    CPPUNIT_ASSERT_MESSAGE("Unexpected min x", std::abs(my_aabb3d.min.x - (-10+15)) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected min y", std::abs(my_aabb3d.min.y - (-11+20)) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected min z", std::abs(my_aabb3d.min.z - -12) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected max x", std::abs(my_aabb3d.max.x - (13+15)) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected max y", std::abs(my_aabb3d.max.y - (14+20)) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected max z", std::abs(my_aabb3d.max.z - 15) < epsilon);
}

void TemplateTest::expandXYTest()
{
    std::stringstream ss;
    my_aabb3d.expandXY(5);
    ss << "Unexpected min x: " << my_aabb3d.min.x << ".";
    CPPUNIT_ASSERT_MESSAGE(ss.str(), std::abs(my_aabb3d.min.x - (-10-5)) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected min y", std::abs(my_aabb3d.min.y - (-11-5)) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected min z", std::abs(my_aabb3d.min.z - -12) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected max x", std::abs(my_aabb3d.max.x - (13+5)) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected max y", std::abs(my_aabb3d.max.y - (14+5)) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Unexpected max z", std::abs(my_aabb3d.max.z - 15) < epsilon);
}


}
