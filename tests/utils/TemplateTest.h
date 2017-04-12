//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TEMPLATE_TEST_H
#define TEMPLATE_TEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

namespace cura
{

class TemplateTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(TemplateTest);
    CPPUNIT_TEST(smokeTest);
    CPPUNIT_TEST_SUITE_END();

public:
    void setUp();

    void tearDown();

    void smokeTest();

private:

};

}

#endif //TEMPLATE_TEST_H

