//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SpaceFillingTreeFillTest.h"

namespace cura
{
CPPUNIT_TEST_SUITE_REGISTRATION(SpaceFillingTreeFillTest);

void SpaceFillingTreeFillTest::setUp()
{
    // no dothing
}

void SpaceFillingTreeFillTest::tearDown()
{
    // no dothing
}

void SpaceFillingTreeFillTest::debugCheck()
{
    bool fail = tree.debugCheck();
    CPPUNIT_ASSERT_MESSAGE("Space filling tree incorrect!", !fail);
}

void SpaceFillingTreeFillTest::boundsCheck()
{
    struct Visitor : public SpaceFillingTree::LocationVisitor
    {
        AABB aabb;
        Visitor()
        {}
        /*!
         * Register a location being crossed during the walk.
         * \param node The node visited
         */
        void visit(const SpaceFillingTree::Node* node)
        {
            aabb.include(node->middle);
        }
    };
    Visitor v;
    tree.walk(v);
    const coord_t expected_tree_width = 2 * radius * (1.0 - pow(0.5, (double)depth)); // fractal depth determines the covered square of the fractal
    std::stringstream ss;
    ss << "Tree is not spanning expected square " << expected_tree_width << "; actual width: " << (v.aabb.max.X - v.aabb.min.X);
    CPPUNIT_ASSERT_MESSAGE(ss.str(), std::abs((v.aabb.max.X - v.aabb.min.X) - expected_tree_width) <= allowed_error);
    CPPUNIT_ASSERT_MESSAGE(ss.str(), std::abs((v.aabb.max.Y - v.aabb.min.Y) - expected_tree_width) <= allowed_error);
    CPPUNIT_ASSERT_MESSAGE("Tree middle is not the given middle", vSize(v.aabb.getMiddle() - middle) <= allowed_error);
}

}
