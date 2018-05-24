//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef CROSS3D_TEST_H
#define CROSS3D_TEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <../src/infill/Cross3D.h>

namespace cura
{

class Cross3DTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(Cross3DTest);
    CPPUNIT_TEST(debugCheck);
    CPPUNIT_TEST_SUITE_END();

public:
    /*!
     * \brief Sets up the test suite to prepare for testing.
     */
    void setUp();

    /*!
     * \brief Tears down the test suite when testing is done.
     */
    void tearDown();

    /*!
     * debugCheck of the tree
     */
    void debugCheck();
private:
    coord_t allowed_error = 10;
};

}

#endif //CROSS3D_TEST_H

