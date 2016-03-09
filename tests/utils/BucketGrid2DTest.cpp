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

void BucketGrid2DTest::findNearbyObjectsFarTest()
{
    std::vector<Point> input;
    input.emplace_back(0, 100);
    const Point target(100, 100);
    std::unordered_set<Point> near;
    std::unordered_set<Point> far;
    far.emplace(0, 100);
    findNearbyObjectsAssert(input, target, 10, near, far);
}

void BucketGrid2DTest::findNearbyObjectsLine2Test()
{
    std::vector<Point> input;
    for (long long x = 0; x < 200; x++)
    {
        input.emplace_back(x, 95);
    }
    const Point target(99, 100); //Slightly shifted.
    const unsigned long long grid_size = 10;
    std::unordered_set<Point> near;
    std::unordered_set<Point> far;
    for (const Point point : input)
    {
        unsigned long long distance = vSize(point - target);
        if (distance < grid_size)
        {
            near.insert(point);
        }
        else if (distance > grid_size * 2) //Grid size * 2 are guaranteed to be considered "far".
        {
            far.insert(point);
        }
    }
    findNearbyObjectsAssert(input, target, grid_size, near, far);
}

void BucketGrid2DTest::findNearbyObjectsLineTest()
{
    std::vector<Point> input;
    for (long long x = 0; x < 200; x++)
    {
        input.emplace_back(x, 95);
    }
    const Point target(100, 100);
    const unsigned long long grid_size = 10;
    std::unordered_set<Point> near;
    std::unordered_set<Point> far;
    for (const Point point : input)
    {
        unsigned long long distance = vSize(point - target);
        if (distance < grid_size)
        {
            near.insert(point);
        }
        else if (distance > grid_size * 2) //Grid size * 2 are guaranteed to be considered "far".
        {
            far.insert(point);
        }
    }
    findNearbyObjectsAssert(input, target, grid_size, near, far);
}

void BucketGrid2DTest::findNearbyObjectsNearTest()
{
    std::vector<Point> input;
    input.emplace_back(95, 100);
    const Point target(100, 100);
    std::unordered_set<Point> near;
    near.emplace(95, 100);
    std::unordered_set<Point> far;
    findNearbyObjectsAssert(input, target, 10, near, far);
}

void BucketGrid2DTest::findNearbyObjectsSameTest()
{
    std::vector<Point> input;
    input.emplace_back(100, 100);
    const Point target(100, 100);
    std::unordered_set<Point> near;
    near.emplace(100, 100);
    std::unordered_set<Point> far;
    findNearbyObjectsAssert(input, target, 10, near, far);
}

void BucketGrid2DTest::findNearbyObjectsAssert(const std::vector<Point>& registered_points, Point target, const unsigned long long grid_size, const std::unordered_set<Point>& expected_near, const std::unordered_set<Point>& expected_far)
{
    BucketGrid2D<Point> grid(grid_size);
    for(Point point : registered_points)
    {
        grid.insert(point, point);
    }

    const std::vector<Point> result = grid.findNearbyObjects(target); //The actual call to test.

    //Note that the result may contain the same point more than once. This test is robust against that.
    for (const Point point : expected_near) //Are all near points reported as near?
    {
        std::stringstream ss;
        ss << "Point " << point << " is near " << target << " (distance " << vSize(point - target) << "), but findNearbyObjects didn't report it as such. Grid size: " << grid_size;
        CPPUNIT_ASSERT_MESSAGE(ss.str(), std::find(result.begin(), result.end(), point) != result.end()); //Must be in result.
    }
    for (const Point point : expected_far) //Are all far points NOT reported as near?
    {
        std::stringstream ss;
        ss << "Point " << point << " is far from " << target << " (distance " << vSize(point - target) << "), but findNearbyObjects thought it was near. Grid size: " << grid_size;
        CPPUNIT_ASSERT_MESSAGE(ss.str(), std::find(result.begin(), result.end(), point) == result.end()); //Must not be in result.
    }
}

}