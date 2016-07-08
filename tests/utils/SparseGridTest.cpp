//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SparseGridTest.h"

#include <algorithm>
#include <vector>

namespace cura
{

CPPUNIT_TEST_SUITE_REGISTRATION(SparseGridTest);

void SparseGridTest::setUp()
{
    //Do nothing.
}

void SparseGridTest::tearDown()
{
    //Do nothing.
}

void SparseGridTest::getNearbyFarTest()
{
    std::vector<Point> input;
    input.emplace_back(0, 100);
    const Point target(100, 100);
    std::unordered_set<Point> near;
    std::unordered_set<Point> far;
    far.emplace(0, 100);
    getNearbyAssert(input, target, 10, near, far);
}

void SparseGridTest::getNearbyLine2Test()
{
    std::vector<Point> input;
    for (coord_t x = 0; x < 200; x++)
    {
        input.emplace_back(x, 95);
    }
    const Point target(99, 100); //Slightly shifted.
    const coord_t grid_size = 10;
    std::unordered_set<Point> near;
    std::unordered_set<Point> far;
    for (const Point point : input)
    {
        coord_t distance = vSize(point - target);
        if (distance < grid_size)
        {
            near.insert(point);
        }
        else if (distance > grid_size * 2) //Grid size * 2 are guaranteed to be considered "far".
        {
            far.insert(point);
        }
    }
    getNearbyAssert(input, target, grid_size, near, far);
}

void SparseGridTest::getNearbyLineTest()
{
    std::vector<Point> input;
    for (coord_t x = 0; x < 200; x++)
    {
        input.emplace_back(x, 95);
    }
    const Point target(100, 100);
    const coord_t grid_size = 10;
    std::unordered_set<Point> near;
    std::unordered_set<Point> far;
    for (const Point point : input)
    {
        coord_t distance = vSize(point - target);
        if (distance < grid_size)
        {
            near.insert(point);
        }
        else if (distance > grid_size * 2) //Grid size * 2 are guaranteed to be considered "far".
        {
            far.insert(point);
        }
    }
    getNearbyAssert(input, target, grid_size, near, far);
}

void SparseGridTest::getNearbyNearTest()
{
    std::vector<Point> input;
    input.emplace_back(95, 100);
    const Point target(100, 100);
    std::unordered_set<Point> near;
    near.emplace(95, 100);
    std::unordered_set<Point> far;
    getNearbyAssert(input, target, 10, near, far);
}

void SparseGridTest::getNearbySameTest()
{
    std::vector<Point> input;
    input.emplace_back(100, 100);
    const Point target(100, 100);
    std::unordered_set<Point> near;
    near.emplace(100, 100);
    std::unordered_set<Point> far;
    getNearbyAssert(input, target, 10, near, far);
}

void SparseGridTest::getNearestChoiceTest()
{
    std::vector<Point> input;
    input.emplace_back(95, 100);
    input.emplace_back(103, 100);
    input.emplace_back(200, 100);
    getNearestAssert(input, Point(100, 100), 10, new Point(103, 100));
}

void SparseGridTest::getNearestEqualTest()
{
    std::vector<Point> registered_points;
    registered_points.emplace_back(95, 100);
    registered_points.emplace_back(105, 100);
    Point target = Point(100, 100);
    const coord_t grid_size = 10;
    const Point expected1 = Point(95, 100);
    const Point expected2 = Point(105, 100);

    SparseGrid<Point> grid(grid_size);
    for (Point point : registered_points)
    {
        grid.insert(point, point);
    }

    typename SparseGrid<Point>::Elem result;
    //The actual call to test.
    const bool success = grid.getNearest(target, grid_size,
                                         result, SparseGrid<Point>::no_precondition);

    {
        std::stringstream ss;
        ss << "getNearest returned " << success << " but should've returned true.";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), success);
    }
    {
        std::stringstream ss;
        ss << "getNearest reported the nearest point to be " << result.val <<
            " (distance " << vSize(target - result.val) <<
            "), but it should've been " << expected1 <<
            " (distance " << vSize(expected1 - target) <<
            ") or " << expected2 <<
            " (distance " << vSize(expected2 - target) << ").";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), result.val == expected1 || result.val == expected2);
    }
}

void SparseGridTest::getNearestFilterTest()
{
    std::vector<Point> input;
    input.emplace_back(95, 100);
    input.emplace_back(98, 100);
    input.emplace_back(106, 100);
    std::function<bool(const typename SparseGrid<Point>::Elem &)> filter =
        [&] (const typename SparseGrid<Point>::Elem& elem) -> bool
        {
            return elem.point.X > 100;
        };
    getNearestAssert(input, Point(100, 100), 10, new Point(106, 100), filter);
}

void SparseGridTest::getNearestNoneTest()
{
    std::vector<Point> input;
    getNearestAssert(input, Point(100, 100), 10, nullptr);
}

void SparseGridTest::getNearestSameTest()
{
    std::vector<Point> input;
    input.emplace_back(100, 100);
    getNearestAssert(input, Point(100, 100), 10, new Point(100, 100));
}

void SparseGridTest::getNearbyAssert(
    const std::vector<Point>& registered_points,
    Point target, const coord_t grid_size,
    const std::unordered_set<Point>& expected_near,
    const std::unordered_set<Point>& expected_far)
{
    SparseGrid<Point> grid(grid_size);
    for(Point point : registered_points)
    {
        grid.insert(point, point);
    }

    //The actual call to test.
    const std::vector<typename SparseGrid<Point>::Elem> result =
        grid.getNearby(target, grid_size);

    //Are all near points reported as near?
    for (const Point point : expected_near)
    {
        std::stringstream ss;
        ss << "Point " << point << " is near " << target <<
            " (distance " << vSize(point - target) <<
            "), but getNearby didn't report it as such. Grid size: " <<
            grid_size;
        //Must be in result.
        CPPUNIT_ASSERT_MESSAGE(
            ss.str(),
            std::find_if(result.begin(), result.end(),
                         [&point](const typename SparseGrid<Point>::Elem &elem)
                         {
                             return elem.val == point;
                         }) !=
            result.end());
    }
    //Are all far points NOT reported as near?
    for (const Point point : expected_far)
    {
        std::stringstream ss;
        ss << "Point " << point << " is far from " << target <<
            " (distance " << vSize(point - target) <<
            "), but getNearby thought it was near. Grid size: " << grid_size;
        //Must not be in result.
        CPPUNIT_ASSERT_MESSAGE(
            ss.str(),
            std::find_if(result.begin(), result.end(),
                         [&point](const typename SparseGrid<Point>::Elem &elem)
                         {
                             return elem.val == point;
                         }) ==
            result.end());
    }
}

void SparseGridTest::getNearestAssert(
    const std::vector<Point>& registered_points,
    Point target, const coord_t grid_size,
    Point* expected,
    const std::function<bool(const typename SparseGrid<Point>::Elem& elem)> &precondition)
{
    SparseGrid<Point> grid(grid_size);
    for (Point point : registered_points)
    {
        grid.insert(point, point);
    }

    typename SparseGrid<Point>::Elem result;
    //The actual call to test.
    const bool success = grid.getNearest(target, grid_size,
                                         result, precondition);

    {
        std::stringstream ss;
        ss << "getNearest returned " << success <<
            " but should've returned " << (expected != nullptr) << ".";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), success == (expected != nullptr));
    }
    if (expected)
    {
        std::stringstream ss;
        ss << "getNearest reported the nearest point to be " << result.val <<
            " (distance " << vSize(target - result.val) <<
            "), but it was " << *expected <<
            " (distance " << vSize(*expected - target) << ").";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), result.val == *expected);
    }
}

}
