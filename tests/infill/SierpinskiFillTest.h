//Copyright (c) 2017 Tim Kuipers
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SIERPINSKI_FILL_TEST_H
#define SIERPINSKI_FILL_TEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <../src/infill/SierpinskiFill.h>

namespace cura
{

class SierpinskiFillTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(SierpinskiFillTest);
    CPPUNIT_TEST(debugCheck);
    CPPUNIT_TEST(boundsCheck);
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

    /*!
     * checking whether all falls within the original square
     */
    void boundsCheck();
private:
    coord_t allowed_error = 10;
};

}

#endif //SIERPINSKI_FILL_TEST_H

