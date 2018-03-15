//Copyright (c) 2018 Ultimaker
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef FRACTAL_INFILL_TEST_H
#define FRACTAL_INFILL_TEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <../src/infill/InfillFractal.h>

namespace cura
{

class FractalInfillTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(FractalInfillTest);
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

    /*!
     * checking whether all falls within the original square
     */
    void boundsCheck();
private:
    coord_t allowed_error = 10;
};

}

#endif //FRACTAL_INFILL_TEST_H

