//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include "../../src/utils/Simplify.h" //The unit under test.

namespace cura
{

class SimplifyTest: public testing::Test
{
public:
    //Settings to use for most of these tests.
    static constexpr coord_t max_resolution = 1000;
    static constexpr coord_t max_deviation = 100;
    static constexpr coord_t max_area_deviation = 20000;

    /*!
     * Use this basic simplify instance to easily run tests.
     */
    Simplify simplifier;

    SimplifyTest() : simplifier(max_resolution, max_deviation, max_area_deviation) {}

    void SetUp()
    {
        simplifier = Simplify(max_resolution, max_deviation, max_area_deviation); //Reset in case the test wants to change a parameter.
    }
};

TEST_F(SimplifyTest, CircleMaxResolution)
{
    Polygon circle;
    constexpr coord_t radius = 100000;
    constexpr double segment_length = max_resolution - 10;
    constexpr double tau = 6.283185307179586476925286766559; //2 * pi.
    constexpr double increment = segment_length / radius; //Segments of 990 units.
    for (double angle = 0; angle < tau; angle += increment)
    {
        circle.add(Point(std::cos(angle) * radius, std::sin(angle) * radius));
    }

    simplifier.max_deviation = 999999; //For this test, the maximum deviation should not be an issue.
    constexpr bool is_closed = true;
    simplifier.polygon(circle);
    //With segments of 990, we need to remove exactly half of the vertices to meet the requirement that all segments are >=1000.
    constexpr coord_t minimum_segment_length = max_resolution;
    constexpr coord_t maximum_segment_length = segment_length * 2 + 20; //+20 for some error margin due to rounding.

    for (size_t point_index = 1; point_index + 1 < circle.size(); point_index++) //Don't check the last vertex. Due to odd-numbered vertices it has to be shorter than the minimum.
    {
        coord_t segment_length = vSize(circle[point_index % circle.size()] - circle[point_index - 1]);
        ASSERT_GE(segment_length, minimum_segment_length) << "Segment " << (point_index - 1) << " - " << point_index << " is too short!";
        ASSERT_LE(segment_length, maximum_segment_length) << "Segment " << (point_index - 1) << " - " << point_index << " is too long!";
    }
}

}