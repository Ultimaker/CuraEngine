//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "MappingFunctionTest.h"

#include <../src/utils/MappingFunction.h>

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(MappingFunctionTest);

void MappingFunctionTest::setUp()
{
    std::vector<float> points;
    points.emplace_back(0.0);
    points.emplace_back(2.0);
    points.emplace_back(3.0);
    mf = new MappingFunction(points, 0.0, 2.0);
}

void MappingFunctionTest::tearDown()
{
    delete mf;
}

void MappingFunctionTest::test_1  ()
{
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, mf->map(-1.0), delta);
}
void MappingFunctionTest::test0   ()
{
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, mf->map(0.0), delta);
}
void MappingFunctionTest::test_25 ()
{
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, mf->map(0.25), delta);
}
void MappingFunctionTest::test_5  ()
{
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, mf->map(0.5), delta);
}
void MappingFunctionTest::test_75 ()
{
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1.5, mf->map(0.75), delta);
}
void MappingFunctionTest::test1   ()
{
    CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, mf->map(1.0), delta);
}
void MappingFunctionTest::test1_25()
{
    CPPUNIT_ASSERT_DOUBLES_EQUAL(2.25, mf->map(1.25), delta);
}
void MappingFunctionTest::test1_5 ()
{
    CPPUNIT_ASSERT_DOUBLES_EQUAL(2.5, mf->map(1.5), delta);
}
void MappingFunctionTest::test1_75()
{
    CPPUNIT_ASSERT_DOUBLES_EQUAL(2.75, mf->map(1.75), delta);
}
void MappingFunctionTest::test2   ()
{
    CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, mf->map(2.0), delta);
}
void MappingFunctionTest::test2_25()
{
    CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, mf->map(2.25), delta);
}
void MappingFunctionTest::test2_5 ()
{
    CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, mf->map(2.5), delta);
}
void MappingFunctionTest::test2_75()
{
    CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, mf->map(2.75), delta);
}
void MappingFunctionTest::test3   ()
{
    CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, mf->map(3.0), delta);
}
void MappingFunctionTest::test5   ()
{
    CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, mf->map(5.0), delta);
}

}
