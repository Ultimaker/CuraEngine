//Copyright (c) 2017 Tim Kuipers
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SPACE_FILLING_TREE_FILL_TEST_H
#define SPACE_FILLING_TREE_FILL_TEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <../src/infill/SpaceFillingTreeFill.h>

namespace cura
{

class SpaceFillingTreeFillTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(SpaceFillingTreeFillTest);
    CPPUNIT_TEST(test);
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
     * testing test suite
     */
    void test();
private:
    
};

}

#endif //SPACE_FILLING_TREE_FILL_TEST_H

