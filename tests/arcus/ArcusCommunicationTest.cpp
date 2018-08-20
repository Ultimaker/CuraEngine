//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ArcusCommunicationTest.h"

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(ArcusCommunicationTest);

    void ArcusCommunicationTest::setUp()
    {

    }

    void ArcusCommunicationTest::tearDown()
    {
        //Do nothing.
    }

    void ArcusCommunicationTest::dummyTest()
    {
        CPPUNIT_ASSERT_EQUAL_MESSAGE("Value 1 2", 1, 2);
    }

}