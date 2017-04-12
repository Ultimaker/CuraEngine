//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef AABB_TEST_H
#define AABB_TEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

namespace cura
{

class AABBTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(AABBTest);
    CPPUNIT_TEST(smokeTest);
    CPPUNIT_TEST_SUITE_END();

public:
    void setUp();

    void tearDown();

    void smokeTest();

private:

};

}

#endif //AABB_TEST_H

