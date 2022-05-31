//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include "../../src/utils/Simplify.h" //The unit under test.
#include "../../src/utils/polygonUtils.h" //Helper functions for testing deviation.

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

    //Some polygons to run tests on.
    Polygon circle; //High resolution circle.

    SimplifyTest() : simplifier(max_resolution, max_deviation, max_area_deviation) {}

    void SetUp()
    {
        simplifier = Simplify(max_resolution, max_deviation, max_area_deviation); //Reset in case the test wants to change a parameter.

        constexpr coord_t radius = 100000;
        constexpr double segment_length = max_resolution - 10;
        constexpr double tau = 6.283185307179586476925286766559; //2 * pi.
        constexpr double increment = segment_length / radius; //Segments of 990 units.
        for(double angle = 0; angle < tau; angle += increment)
        {
            circle.add(Point(std::cos(angle) * radius, std::sin(angle) * radius));
        }
    }
};

/*!
 * Test the maximum resolution being held.
 *
 * If the deviation is practically unlimited, there should be no line segments
 * smaller than the maximum resolution in the result.
 */
TEST_F(SimplifyTest, CircleMaxResolution)
{
    simplifier.max_deviation = 999999; //For this test, the maximum deviation should not be an issue.
    circle = simplifier.polygon(circle);

    for(size_t point_index = 1; point_index + 1 < circle.size(); point_index++) //Don't check the last vertex. Due to odd-numbered vertices it has to be shorter than the minimum.
    {
        coord_t segment_length = vSize(circle[point_index % circle.size()] - circle[point_index - 1]);
        EXPECT_GE(segment_length, max_resolution) << "Segment " << (point_index - 1) << " - " << point_index << " is too short! May not be less than the maximum resolution.";
    }
}

/*!
 * Test the maximum deviation being held.
 *
 * The deviation may not exceed the maximum deviation. If the maximum resolution
 * is high, all line segments should get considered for simplification. It
 * should then approach the maximum deviation regularly.
 */
TEST_F(SimplifyTest, CircleMaxDeviation)
{
    simplifier.max_resolution = 999999; //For this test, the maximum resolution should not be an issue.
    Polygon simplified = simplifier.polygon(circle);

    //Check on each vertex if it didn't deviate too much.
    for(Point v : circle)
    {
        Point moved_point = v;
        PolygonUtils::moveInside(simplified, moved_point);
        const coord_t deviation = vSize(moved_point - v);
        EXPECT_LE(deviation, simplifier.max_deviation);
    }
    //Also check the other way around, since the longest distance may also be on a vertex of the new polygon.
    for(Point v : simplified)
    {
        Point moved_point = v;
        PolygonUtils::moveInside(circle, moved_point);
        const coord_t deviation = vSize(moved_point - v);
        EXPECT_LE(deviation, simplifier.max_deviation);
    }
}

}