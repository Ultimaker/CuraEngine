//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h> //To run the tests.
#include "../src/PathOrderOptimizer.h" //The code under test.

namespace cura
{

class PathOrderOptimizerTest : public testing::Test
{
public:
    /*
     * A blank optimizer with no polygons added yet.
     */
    PathOrderOptimizer<ConstPolygonRef> empty;

    PathOrderOptimizerTest() : empty(Point(0, 0)) {}

    void SetUp()
    {
        empty = PathOrderOptimizer<ConstPolygonRef>(Point(0, 0));
    }
};

/*!
 * Test optimizing an empty set of paths.
 */
TEST_F(PathOrderOptimizerTest, OptimizeWhileEmpty)
{
    empty.optimize(); //Don't crash.
    EXPECT_EQ(empty.paths.size(), 0) << "Still empty!";
}

}