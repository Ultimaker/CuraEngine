//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "AABBTest.h"

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(AABBTest);

void AABBTest::setUp()
{
}

void AABBTest::tearDown()
{
    //Do nothing.
}


void AABBTest::smokeTest()
{
    CPPUNIT_ASSERT_MESSAGE("Boom!", 1 < 5);
}


}
