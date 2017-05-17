//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TEMPLATE_TEST_H
#define TEMPLATE_TEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include <../src/utils/AABB3D.h>

namespace cura
{

class TemplateTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(TemplateTest);
    CPPUNIT_TEST(smokeTest);
    CPPUNIT_TEST(hitTest);
    CPPUNIT_TEST(hitTest2);
    CPPUNIT_TEST(hitTest3);
    CPPUNIT_TEST(hitTest4);
    CPPUNIT_TEST(includeTest);
    CPPUNIT_TEST(includeZTest);
    CPPUNIT_TEST(offsetTest);
    CPPUNIT_TEST(offsetTest2);
    CPPUNIT_TEST(expandXYTest);
    CPPUNIT_TEST_SUITE_END();

public:
    void setUp();

    void tearDown();

    void smokeTest();
    void hitTest();
    void hitTest2();
    void hitTest3();
    void hitTest4();
    void includeTest();
    void includeZTest();
    void offsetTest();
    void offsetTest2();
    void expandXYTest();

private:
    /*!
     * \brief The maximum allowed error in floats.
     */
    static constexpr double epsilon = 0.000001f;

    AABB3D my_aabb3d;
};

}

#endif //TEMPLATE_TEST_H

