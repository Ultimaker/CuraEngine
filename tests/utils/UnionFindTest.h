//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UNIONFINDTEST_H
#define UNIONFINDTEST_H

#include <cppunit/TestFixture.h> //To create unit tests.
#include <cppunit/extensions/HelperMacros.h> //For defining tests and test suites.

#include "../src/utils/UnionFind.h" //The class we're testing.

namespace cura
{

class UnionFindTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(UnionFindTest);
    CPPUNIT_TEST(findSimpleTest);
    CPPUNIT_TEST(findMultipleTest);
    CPPUNIT_TEST(uniteTwoTest);
    CPPUNIT_TEST(uniteThreeTest);
    CPPUNIT_TEST(uniteSetsTest);
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

    //The actual test cases.
    void findSimpleTest(); //Simple adding and finding back.
    void findMultipleTest(); //Finding between multiple items.
    void uniteTwoTest(); //Uniting two items.
    void uniteThreeTest(); //Uniting three items.
    void uniteSetsTest(); //Uniting sets that already have multiple items.

private:
    UnionFind<char> union_find; //An empty union find for easy use by tests.
};

}
#endif /* UNIONFINDTEST_H */

