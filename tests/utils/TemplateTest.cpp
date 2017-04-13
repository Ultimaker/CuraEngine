//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.
//Use this file as a template for testing new classes
#include "TemplateTest.h"

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(TemplateTest);

void TemplateTest::setUp()
{
}

void TemplateTest::tearDown()
{
}


void TemplateTest::smokeTest()
{
    CPPUNIT_ASSERT_MESSAGE("Boom!", 1 < 5);
}


}
