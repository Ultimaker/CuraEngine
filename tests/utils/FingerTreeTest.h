//Copyright (c) 2017 Ultimaker
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef FINGER_TREE_TEST_H
#define FINGER_TREE_TEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <../src/utils/FingerTree.h>

namespace cura
{

class FingerTreeTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(FingerTreeTest);
    CPPUNIT_TEST(testDepth);
    CPPUNIT_TEST(copyTest);
    CPPUNIT_TEST_SUITE_END();

public:
    /*!
     * \brief Sets up the test suite to prepare for testing.
     * 
     * Since <em>FingerTreeTest</em> only has static functions, no instance
     * needs to be created here.
     */
    void setUp();

    /*!
     * \brief Tears down the test suite when testing is done.
     * 
     * Since <em>FingerTreeTest</em> only has static functions, no instance
     * exists that needs to be destroyed.
     */
    void tearDown();

    //! Test whether the tree doesn't make unneeded copies
    void copyTest();
    
    //! Test whether the depth iterators and construction work as expected.
    void testDepth();

private:
};

}

#endif //FINGER_TREE_TEST_H

