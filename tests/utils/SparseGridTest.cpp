//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>
#include <algorithm>
#include <unordered_set>
#include <vector>

#include "../src/utils/SparseGrid.h"
#include "../src/utils/SparsePointGridInclusive.h"

namespace cura
{

struct GetNearbyParameters
{
    std::vector<Point> registered_points;
    std::unordered_set<Point> expected_near;
    std::unordered_set<Point> expected_far;

    GetNearbyParameters(const std::vector<Point> registered_points, const std::unordered_set<Point> expected_near, const std::unordered_set<Point> expected_far)
    : registered_points(registered_points)
    , expected_near(expected_near)
    , expected_far(expected_far)
    {
    }
};

class GetNearbyTest : public testing::TestWithParam<GetNearbyParameters>
{
};

TEST_P(GetNearbyTest, GetNearby)
{
    const Point target(100, 100);
    constexpr coord_t grid_size = 10;
    const GetNearbyParameters parameters = GetParam();
    SparsePointGridInclusive<Point> grid(grid_size);
    for (const Point point : parameters.registered_points)
    {
        grid.insert(point, point);
    }

    const std::vector<typename SparsePointGridInclusive<Point>::Elem> result = grid.getNearby(target, grid_size);

    //Are all near points reported as near?
    for (const Point point : parameters.expected_near)
    {
        EXPECT_NE(result.end(), std::find_if(result.begin(), result.end(), [&point](const typename SparsePointGridInclusive<Point>::Elem &elem) { return elem.val == point; }))
            << "Point " << point << " is near " << target << " (distance " << vSize(point - target) << "), but getNearby didn't find it. Grid size: " << grid_size;
    }
    //Are all far points NOT reported as near?
    for (const Point point : parameters.expected_far)
    {
        EXPECT_EQ(result.end(), std::find_if(result.begin(), result.end(), [&point](const typename SparsePointGridInclusive<Point>::Elem &elem) { return elem.val == point; }))
            << "Point " << point << " is far from " << target << " (distance " << vSize(point - target) << "), but getNearby thought it was near. Grid size: " << grid_size;
    }
}

INSTANTIATE_TEST_CASE_P(GetNearbyInstantiation, GetNearbyTest, testing::Values(
    GetNearbyParameters({ Point(0,   100) }, std::unordered_set<Point>(), std::unordered_set<Point>({ Point(0,   100) })), //A far point.
    GetNearbyParameters({ Point(95,  100) }, std::unordered_set<Point>({ Point(95,  100) }), std::unordered_set<Point>()), //A near point.
    GetNearbyParameters({ Point(100, 100) }, std::unordered_set<Point>({ Point(100, 100) }), std::unordered_set<Point>())  //On top of the target.
));

TEST_F(GetNearbyTest, getNearbyLine2)
{
    std::vector<Point> input;
    for (coord_t x = 0; x < 200; x++)
    {
        input.emplace_back(x, 95);
    }
    const Point target(99, 100); //Slightly shifted.
    constexpr coord_t grid_size = 10;
    std::unordered_set<Point> near;
    std::unordered_set<Point> far;
    for (const Point point : input)
    {
        const coord_t distance = vSize(point - target);
        if (distance < grid_size)
        {
            near.insert(point);
        }
        else if (distance > grid_size * 2) //Grid size * 2 are guaranteed to be considered "far".
        {
            far.insert(point);
        }
    }

    SparsePointGridInclusive<Point> grid(grid_size);
    for (const Point point : input)
    {
        grid.insert(point, point);
    }
    const std::vector<typename SparsePointGridInclusive<Point>::Elem> result = grid.getNearby(target, grid_size);

    //Are all near points reported as near?
    for (const Point point : near)
    {
        EXPECT_NE(result.end(), std::find_if(result.begin(), result.end(), [&point](const typename SparsePointGridInclusive<Point>::Elem &elem) { return elem.val == point; }))
            << "Point " << point << " is near " << target << " (distance " << vSize(point - target) << "), but getNearby didn't find it. Grid size: " << grid_size;
    }
    //Are all far points NOT reported as near?
    for (const Point point : far)
    {
        EXPECT_EQ(result.end(), std::find_if(result.begin(), result.end(), [&point](const typename SparsePointGridInclusive<Point>::Elem &elem) { return elem.val == point; }))
            << "Point " << point << " is far from " << target << " (distance " << vSize(point - target) << "), but getNearby thought it was near. Grid size: " << grid_size;
    }
}

TEST_F(GetNearbyTest, getNearbyLine)
{
    std::vector<Point> input;
    for (coord_t x = 0; x < 200; x++)
    {
        input.emplace_back(x, 95);
    }
    const Point target(100, 100);
    constexpr coord_t grid_size = 10;
    std::unordered_set<Point> near;
    std::unordered_set<Point> far;
    for (const Point point : input)
    {
        const coord_t distance = vSize(point - target);
        if (distance < grid_size)
        {
            near.insert(point);
        }
        else if (distance > grid_size * 2) //Grid size * 2 are guaranteed to be considered "far".
        {
            far.insert(point);
        }
    }

    SparsePointGridInclusive<Point> grid(grid_size);
    for (const Point point : input)
    {
        grid.insert(point, point);
    }
    const std::vector<typename SparsePointGridInclusive<Point>::Elem> result = grid.getNearby(target, grid_size);

    //Are all near points reported as near?
    for (const Point point : near)
    {
        EXPECT_NE(result.end(), std::find_if(result.begin(), result.end(), [&point](const typename SparsePointGridInclusive<Point>::Elem &elem) { return elem.val == point; }))
            << "Point " << point << " is near " << target << " (distance " << vSize(point - target) << "), but getNearby didn't find it. Grid size: " << grid_size;
    }
    //Are all far points NOT reported as near?
    for (const Point point : far)
    {
        EXPECT_EQ(result.end(), std::find_if(result.begin(), result.end(), [&point](const typename SparsePointGridInclusive<Point>::Elem &elem) { return elem.val == point; }))
            << "Point " << point << " is far from " << target << " (distance " << vSize(point - target) << "), but getNearby thought it was near. Grid size: " << grid_size;
    }
}

struct GetNearestParameters
{
    std::vector<Point> registered_points;
    Point* result;
    std::function<bool(const typename SparsePointGridInclusive<Point>::Elem&)> filter;

    GetNearestParameters(const std::vector<Point> registered_points, Point* result, const std::function<bool(const typename SparsePointGridInclusive<Point>::Elem&)>& filter = SparsePointGridInclusive<Point>::no_precondition)
    : registered_points(registered_points)
    , result(result)
    , filter(filter)
    {
    }
};

class GetNearestTest : public testing::TestWithParam<GetNearestParameters>
{
};

TEST_P(GetNearestTest, GetNearest)
{
    const GetNearestParameters parameters = GetParam();
    constexpr coord_t grid_size = 10;
    const Point target(100, 100);

    SparsePointGridInclusive<Point> grid(grid_size);
    for (Point point : parameters.registered_points)
    {
        grid.insert(point, point);
    }

    typename SparsePointGridInclusive<Point>::Elem result;
    const bool success = grid.getNearest(target, grid_size, result, parameters.filter);

    ASSERT_EQ(success, parameters.result != nullptr) << "getNearest returned " << success << " but should've returned " << (parameters.result != nullptr) << ".";
    if (parameters.result)
    {
        ASSERT_EQ(result.val, *parameters.result) << "getNearest reported the nearest point to be " << result.val << " (distance " << vSize(target - result.val) << "), but it was " << *parameters.result << " (distance " << vSize(target - *parameters.result) << ").";
    }
}

INSTANTIATE_TEST_CASE_P(GetNearestInstantiation, GetNearestTest, testing::Values(
    GetNearestParameters(std::vector<Point>({ Point(95, 100), Point(103, 100), Point(200, 100) }), new Point(103, 100)), //Choose nearest out of 3 points.
    GetNearestParameters(std::vector<Point>({ Point(95, 100), Point(98, 100), Point(106, 100) }), new Point(106, 100), [](const typename SparsePointGridInclusive<Point>::Elem& elem) -> bool { return elem.point.X > 100; }), //With a filter.
    GetNearestParameters(std::vector<Point>(), nullptr), //No points, no answer.
    GetNearestParameters(std::vector<Point>({ Point(100, 100) }), new Point(100, 100)) //Same point as target.
));

TEST_F(GetNearestTest, Equal)
{
    std::vector<Point> registered_points;
    registered_points.emplace_back(95, 100);
    registered_points.emplace_back(105, 100);
    const Point target = Point(100, 100);
    constexpr coord_t grid_size = 10;
    const Point expected1 = Point(95, 100);
    const Point expected2 = Point(105, 100);

    SparsePointGridInclusive<Point> grid(grid_size);
    for (const Point point : registered_points)
    {
        grid.insert(point, point);
    }

    typename SparsePointGridInclusive<Point>::Elem result;
    //The actual call to test.
    const bool success = grid.getNearest(target, grid_size, result, SparsePointGridInclusive<Point>::no_precondition);

    ASSERT_TRUE(success);
    ASSERT_TRUE(result.val == expected1 || result.val == expected2) << "getNearest reported the nearest point to be " << result.val << " (distance " << vSize(target - result.val) <<
            "), but it should've been " << expected1 << "(distance " << vSize(expected1 - target) <<
            ") or " << expected2 << " (distance " << vSize(expected2 - target) << ").";
}

}
