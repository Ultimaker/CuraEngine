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
    Point middle = Point(0, 0);
    coord_t radius = 123;
    int depth = 4;
    SpaceFillingTree tree = SpaceFillingTree(middle, radius, depth);
};

}

#endif //SPACE_FILLING_TREE_FILL_TEST_H

