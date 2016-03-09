//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "BucketGrid2DTest.h"

#include <../src/utils/BucketGrid2D.h>

#include <algorithm>
#include <vector>

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(BucketGrid2DTest);

void BucketGrid2DTest::setUp()
{
    //Do nothing.
}

void BucketGrid2DTest::tearDown()
{
    //Do nothing.
}

void BucketGrid2DTest::findNearbyObjectsSameTest()
{
    std::vector<Point> input;
    input.emplace_back(100, 100);
    const Point target(100, 100);
    std::unordered_set<Point> output;
    output.emplace(100, 100);
    findNearbyObjectsAssert(input, target, 10, output);
}

void BucketGrid2DTest::findNearbyObjectsAssert(const std::vector<Point>& registered_points, Point target, const unsigned long long grid_size, const std::unordered_set<Point>& expected)
{
    BucketGrid2D<Point> grid(grid_size);
    for(Point point : registered_points)
    {
        grid.insert(point, point);
    }

    const std::vector<Point> result = grid.findNearbyObjects(target); //The actual call to test.

    //Test bijective equality.
    //Note that the result may contain the same point more than once. This test is robust against that (don't check the size!).
    for(const Point point : result)
    {
        std::stringstream ss;
        ss << "Point " << point << " was reported as being close to " << target << ", but shouldn't be. The actual distance was " << vSize(point - target) << " with grid size " << grid_size << ".";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), expected.find(point) != expected.end());
    }
    for(const Point point : expected)
    {
        std::stringstream ss;
        ss << "Point " << point << " was not reported as being close to " << target << ", but it should have been. The actual distance was " << vSize(point - target) << " with grid size " << grid_size << ".";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), std::find(result.begin(), result.end(), point) != result.end());
    }
}

}