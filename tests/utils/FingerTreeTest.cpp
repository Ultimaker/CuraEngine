//Copyright (c) 2017 Ultimaker
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "FingerTreeTest.h"

#include <../src/utils/FingerTree.h>

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(FingerTreeTest);

void FingerTreeTest::setUp()
{
    //Do nothing.
}

void FingerTreeTest::tearDown()
{
    //Do nothing.
}

void FingerTreeTest::copyTest()
{
    struct NoCopy
    {
        bool is_bogus;
        NoCopy(bool is_bogus)
        : is_bogus(is_bogus)
        {}
        NoCopy(const NoCopy& rhs)
        : is_bogus(rhs.is_bogus)
        {
            if (!is_bogus)
            {
                CPPUNIT_ASSERT_MESSAGE(std::string("FingerTree copies element while it should have reserved enough!"), false);
            }
        }
    };
    
    using Node = FingerTree<NoCopy, 2>::Node;
    FingerTree<NoCopy, 2> ft(7, NoCopy(true)); // default value is allowed to be copied!
    /*
     * Create the following tree:
     *      0
     *   1      2
     * 3   4  5   6
     */
    ft.setRoot(NoCopy(false));
    Node root = ft.getRoot();
    for (Node child = root.begin(); child != root.end(); ++child)
    {
        *child = NoCopy(false);
        for (NoCopy& grand_child : child)
        {
            grand_child = NoCopy(false);
        }
    }
}
void FingerTreeTest::testDepth()
{
    FingerTree<int, 4> ft;
    ft.setRoot(0);
    using Node = FingerTree<int, 4>::Node;
    Node root = ft.getRoot();
    for (int& child : root)
        child = 1;
    for (Node child = root.begin(); child != root.end(); ++child)
        for (int& grand_child : child)
            grand_child = 2;
    for (Node child = root.begin(); child != root.end(); ++child)
        for (Node grand_child = child.begin(); grand_child != child.end(); ++grand_child)
            for (int& grand_grand_child : grand_child)
                grand_grand_child = 3;
    
    for (int depth = 0; depth < 4; depth++)
    {
        for (Node n = ft.begin(depth); n != ft.end(depth); ++n)
        {
            char buffer[256];
            sprintf(buffer, "FingerTree depth iterator and construction don't match! constructed depth = %d while iterating depth = %d", *n, depth);
            CPPUNIT_ASSERT_MESSAGE(std::string(buffer), *n == depth);
        }
    }
}

}
