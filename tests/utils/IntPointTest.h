//Copyright (c) 2017 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INT_POINT_TEST_H
#define INT_POINT_TEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <../src/utils/IntPoint.h>

namespace cura
{

class IntPointTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(IntPointTest);
    CPPUNIT_TEST(testRotationMatrix);
    CPPUNIT_TEST_SUITE_END();

public:
    /*!
     * \brief Sets up the test suite to prepare for testing.
     * 
     * Since <em>IntPointTest</em> only has static functions, no instance
     * needs to be created here.
     */
    void setUp();

    /*!
     * \brief Tears down the test suite when testing is done.
     * 
     * Since <em>IntPointTest</em> only has static functions, no instance
     * exists that needs to be destroyed.
     */
    void tearDown();

    //These are the actual test cases. The name of the function sort of describes what it tests but I refuse to document all of these, sorry.
    void testRotationMatrix();

private:
};

}

#endif //INT_POINT_TEST_H

