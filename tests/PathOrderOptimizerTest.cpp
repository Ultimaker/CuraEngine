// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PathOrderOptimizer.h" //The code under test.

#include <gtest/gtest.h> //To run the tests.

// NOLINTBEGIN(*-magic-numbers)
namespace cura
{
// NOLINTBEGIN(misc-non-private-member-variables-in-classes)
class PathOrderOptimizerTest : public testing::Test
{
public:
    /*!
     * A simple isosceles triangle. Base length and height 50.
     */
    Polygon triangle;

    PathOrderOptimizerTest()
    {
    }

    void SetUp() override
    {
        triangle.clear();
        triangle.push_back(Point2LL(0, 0));
        triangle.push_back(Point2LL(50, 0));
        triangle.push_back(Point2LL(25, 50));
    }
};
// NOLINTEND(misc-non-private-member-variables-in-classes)

/*!
 * Test optimizing an empty set of paths.
 */
TEST_F(PathOrderOptimizerTest, OptimizeWhileEmpty)
{
    PathOrderOptimizer<const Polygon*> optimizer(Point2LL(0, 0));
    optimizer.optimize(); // Don't crash.
    EXPECT_EQ(optimizer.paths_.size(), 0) << "Still empty!";
}

/*!
 * Tests traversing three triangles, positioned in a line. The order of
 * traversal should be very clear, from close to the origin to far.
 */
TEST_F(PathOrderOptimizerTest, ThreeTrianglesShortestOrder)
{
    PathOrderOptimizer<const Polygon*> optimizer(Point2LL(0, 0));

    Polygon near = triangle; // Copy, then translate.
    near.translate(Point2LL(100, 100));
    Polygon middle = triangle;
    middle.translate(Point2LL(500, 500));
    Polygon far = triangle;
    far.translate(Point2LL(1000, 1000));

    // Add them out of order so that it's clear that the optimization changes the order.
    optimizer.addPolygon(&middle);
    optimizer.addPolygon(&far);
    optimizer.addPolygon(&near);

    optimizer.optimize();

    EXPECT_EQ(optimizer.paths_[0].vertices_->front(), Point2LL(100, 100)) << "Nearest triangle first.";
    EXPECT_EQ(optimizer.paths_[1].vertices_->front(), Point2LL(500, 500)) << "Middle triangle second.";
    EXPECT_EQ(optimizer.paths_[2].vertices_->front(), Point2LL(1000, 1000)) << "Far triangle last.";
}

} // namespace cura
// NOLINTEND(*-magic-numbers)
