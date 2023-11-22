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
     * A blank optimizer with no polygons added yet. Fresh and virgin.
     */
    PathOrderOptimizer<ConstPolygonPointer> optimizer;

    /*!
     * A simple isosceles triangle. Base length and height 50.
     */
    Polygon triangle;

    PathOrderOptimizerTest() : optimizer(Point2LL(0, 0))
    {
    }

    void SetUp() override
    {
        optimizer = PathOrderOptimizer<ConstPolygonPointer>(Point2LL(0, 0));

        triangle.clear();
        triangle.add(Point2LL(0, 0));
        triangle.add(Point2LL(50, 0));
        triangle.add(Point2LL(25, 50));
    }
};
// NOLINTEND(misc-non-private-member-variables-in-classes)

/*!
 * Test optimizing an empty set of paths.
 */
TEST_F(PathOrderOptimizerTest, OptimizeWhileEmpty)
{
    optimizer.optimize(); // Don't crash.
    EXPECT_EQ(optimizer.paths.size(), 0) << "Still empty!";
}

/*!
 * Tests traversing three triangles, positioned in a line. The order of
 * traversal should be very clear, from close to the origin to far.
 */
TEST_F(PathOrderOptimizerTest, ThreeTrianglesShortestOrder)
{
    Polygon near = triangle; // Copy, then translate.
    near.translate(Point2LL(100, 100));
    Polygon middle = triangle;
    middle.translate(Point2LL(500, 500));
    Polygon far = triangle;
    far.translate(Point2LL(1000, 1000));

    // Add them out of order so that it's clear that the optimization changes the order.
    optimizer.addPolygon(middle);
    optimizer.addPolygon(far);
    optimizer.addPolygon(near);

    optimizer.optimize();

    EXPECT_EQ(optimizer.paths[0].vertices->front(), Point2LL(100, 100)) << "Nearest triangle first.";
    EXPECT_EQ(optimizer.paths[1].vertices->front(), Point2LL(500, 500)) << "Middle triangle second.";
    EXPECT_EQ(optimizer.paths[2].vertices->front(), Point2LL(1000, 1000)) << "Far triangle last.";
}

} // namespace cura
// NOLINTEND(*-magic-numbers)
