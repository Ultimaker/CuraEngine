//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MINIMUM_SPANNING_TREE_TEST_H
#define MINIMUM_SPANNING_TREE_TEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "../src/utils/MinimumSpanningTree.h"

namespace cura
{

class MinimumSpanningTreeTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(MinimumSpanningTreeTest);
    CPPUNIT_TEST(testAllPoints);
    CPPUNIT_TEST(testLeaves);
    CPPUNIT_TEST(testNeighbors);
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

    //These are the actual test cases. The name of the function sort of describes what it tests but I refuse to document all of these, sorry.
    void testAllPoints();
    void testLeaves();
    void testNeighbors();

private:
    std::vector<Point> points_;
    std::vector<int> leaves_;
    std::vector<std::vector<int>> edges_;
    MinimumSpanningTree tree_;
};

}

#endif //MINIMUM_SPANNING_TREE_TEST_H

