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
    //And some polylines.
    Polygon sine; //A sinewave.
    Polygon spiral; //A spiral with gradually increasing segment length.
    Polygon zigzag; //Sawtooth zig-zag pattern.

    SimplifyTest() : simplifier(max_resolution, max_deviation, max_area_deviation) {}

    void SetUp()
    {
        simplifier = Simplify(max_resolution, max_deviation, max_area_deviation); //Reset in case the test wants to change a parameter.

        circle.clear();
        coord_t radius = 100000;
        constexpr double segment_length = max_resolution - 10;
        constexpr double tau = 6.283185307179586476925286766559; //2 * pi.
        const double increment = segment_length / radius; //Segments of 990 units.
        for(double angle = 0; angle < tau; angle += increment)
        {
            circle.add(Point(std::cos(angle) * radius, std::sin(angle) * radius));
        }

        sine.clear();
        const AngleRadians sine_step = 0.01; //How far to continue along the sine curve at every vertex.
        constexpr coord_t amplitude = 45;
        constexpr coord_t y_step = 100;
        constexpr size_t periods = 10; //How many waves of the sine to construct.
        for(double current_sine = 0; current_sine < M_PI * periods; current_sine += sine_step)
        {
            sine.add(Point(std::sin(current_sine) * amplitude, y_step * sine.size()));
        }

        spiral.clear();
        const AngleRadians angle_step = 0.1; //Rotate every next vertex by this amount of radians.
        constexpr coord_t radius_step = 100; //Increase the radius by this amount every vertex.
        constexpr coord_t vertex_count = 1000;
        AngleRadians angle = 0;
        radius = 0;
        for(size_t i = 0; i < vertex_count; ++i)
        {
            spiral.add(Point(std::cos(angle) * radius, std::sin(angle) * radius));
            angle += angle_step;
            radius += radius_step;
        }

        zigzag.clear();
        constexpr coord_t invfreq = 30;
        for(size_t i = 0; i < vertex_count; ++i)
        {
            zigzag.add(Point(-amplitude + (i % 2) * 2 * amplitude, i * invfreq));
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

/*!
 * Test a zig-zagging line where all line segments are considered short.
 *
 * The line zig-zags in a sawtooth pattern, but within a band of a width smaller
 * than the deviation. When all line segments are short, they can all be removed
 * and turned into one straight line.
 */
TEST_F(SimplifyTest, Zigzag)
{
    simplifier.max_resolution = 9999999;
    Polygon simplified = simplifier.polyline(zigzag);
    EXPECT_EQ(simplified.size(), 2) << "All zigzagged lines can be erased because they deviate less than the maximum deviation, leaving only the endpoints.";
}

/*!
 * Test simplifying a spiral where the line segments gradually increase in
 * segment length.
 *
 * The simplification should only be applied to segments that were shorter than
 * the maximum resolution.
 */
TEST_F(SimplifyTest, LimitedLength)
{
    simplifier.max_deviation = 999999; //Maximum deviation should have no effect.
    //Find from where on the segments become longer than the maximum resolution.
    size_t limit_vertex;
    for(limit_vertex = 1; limit_vertex < spiral.size(); ++limit_vertex)
    {
        if(vSize2(spiral[limit_vertex] - spiral[limit_vertex - 1]) > simplifier.max_resolution * simplifier.max_resolution)
        {
            limit_vertex--;
            break;
        }
    }

    Polygon simplified = simplifier.polyline(spiral);

    //Look backwards until the limit vertex is reached to verify that the polygon is unaltered there.
    for(size_t i = 0; i < simplified.size(); ++i)
    {
        size_t vertex_spiral = spiral.size() - 1 - i;
        size_t vertex_simplified = simplified.size() - 1 - i;
        if(vertex_spiral < limit_vertex)
        {
            break; //Things are allowed to be simplified from here.
        }
        EXPECT_EQ(spiral[vertex_spiral], simplified[vertex_simplified]) << "Where line segments are longer than max_resolution, vertices should not be altered.";
    }
}

/*!
 * Test simplifying a zig-zag pattern where the deviation gradually increases.
 *
 * When the deviation is less than max_deviation, it should remove vertices.
 * When it's greater, it should not remove vertices.
 */
TEST_F(SimplifyTest, LimitedError)
{
    simplifier.max_resolution = 9999999;

    //Generate a zig-zag with gradually increasing deviation.
    Polygon increasing_zigzag;
    increasing_zigzag.add(Point(0, 0));
    constexpr coord_t amplitude_step = 1; //Every 2 vertices, the amplitude increases by this much.
    constexpr coord_t y_step = 100;
    const coord_t amplitude_limit = simplifier.max_deviation * 2; //Increase amplitude up to this point. About half of the vertices should get removed.
    for(coord_t amplitude = 0; amplitude < amplitude_limit; amplitude += amplitude_step)
    {
        increasing_zigzag.add(Point(amplitude, increasing_zigzag.size() * y_step));
        increasing_zigzag.add(Point(0, increasing_zigzag.size() * y_step));
    }

    size_t limit_vertex = 2 * simplifier.max_deviation / amplitude_step + 2; //2 vertices per zag. Deviation/step zags. Add 2 since deviation equal to max is allowed.

    Polygon simplified = simplifier.polyline(increasing_zigzag);

    //Look backwards until the limit vertex is reached to verify that the polygon is unaltered there.
    for(size_t i = 0; i < simplified.size(); ++i)
    {
        size_t vertex_zigzag = increasing_zigzag.size() - 1 - i;
        size_t vertex_simplified = simplified.size() - 1 - i;
        if(vertex_zigzag < limit_vertex)
        {
            break; //Things are allowed to be simplified from here.
        }
        EXPECT_EQ(increasing_zigzag[vertex_zigzag], simplified[vertex_simplified]) << "Where line segments are deviating more than max_deviation, vertices should not be altered.";
    }
}

/*!
 * Test simplifying a polyline with small edges encompassed by long edges.
 *
 * The small edges should be removed, but the long edges may not be shifted. The
 * only correct solution involves a vertex that wasn't in the original polyline.
 */
TEST_F(SimplifyTest, LongEdgesNotMoved)
{
    Polygon polyline;
    polyline.add(Point(0, 0));
    polyline.add(Point(10000, 10000)); //Long edge.
    polyline.add(Point(10010, 10000)); //Short edge.
    polyline.add(Point(21010, 0)); //Long edge.

    Polygon simplified = simplifier.polyline(polyline);

    //Verify that all small segments are removed.
    for(size_t i = 1; i < simplified.size(); ++i)
    {
        EXPECT_GE(vSize(simplified[i] - simplified[i - 1]), simplifier.max_resolution) << "There may not be any segment smaller than max resolution.";
    }

    //Verify that all long segments are still present.
    for(size_t i = 0; i < polyline.size() - 1; ++i)
    {
        if(vSize(polyline[i] - polyline[i + 1]) > simplifier.max_resolution)
        {
            //Both endpoints of this line segment must have a distance to the simplified polygon of 0, theoretically.
            //Due to rounding errors we'll allow up to 1 unit.
            Point moved_point = polyline[i];
            PolygonUtils::moveInside(simplified, moved_point);
            const coord_t deviation = vSize(moved_point - polyline[i]);
            EXPECT_LE(deviation, 1) << "The endpoints of long segments must still be in the simplified result.";
        }
    }
}

/*!
 * Tests the case where there are small edges between long edges, where it would
 * normally extend the long edges to prevent shifting them. But extending them
 * to their intersection point would create a vertex that deviates too much from
 * the original shape, so the polygons then can't be simplified.
 */
TEST_F(SimplifyTest, LongEdgesButTooMuchDeviation)
{
    Polygon polyline;
    polyline.add(Point(0, 0));
    polyline.add(Point(0, 10000)); //Long edge.
    polyline.add(Point(10, 10000)); //Short edge.
    polyline.add(Point(20, 0)); //Long edge. Intersection with previous long edge is at 0,20000, which is too far.

    Polygon simplified = simplifier.polyline(polyline);

    //Verify that the polyline is unchanged.
    ASSERT_EQ(polyline.size(), simplified.size()) << "The polyline may not have been simplified because that would introduce vertices that deviate too much.";
    for(size_t i = 0; i < polyline.size(); ++i)
    {
        EXPECT_EQ(polyline[i], simplified[i]) << "The position of the vertices may not have been altered since the polyline was not simplified.";
    }

    polyline.pop_back();
    polyline.add(Point(10, 0)); //Replace last vertex with one that makes the two long edges exactly parallel.

    simplified = simplifier.polyline(polyline);

    //Verify that the polyline is again unchanged.
    ASSERT_EQ(polyline.size(), simplified.size()) << "The polyline may not have been simplified because that would introduce vertices that deviate too much.";
    for(size_t i = 0; i < polyline.size(); ++i)
    {
        EXPECT_EQ(polyline[i], simplified[i]) << "The position of the vertices may not have been altered since the polyline was not simplified.";
    }
}

/*!
 * Test simplifying a sine wave with an amplitude lower than the deviation.
 *
 * The sine wave should get simplified to a line then.
 *
 * This test is similar to LimitedError, but with a higher resolution curve to
 * start with, requiring the algorithm to aggregate errors.
 */
TEST_F(SimplifyTest, Sine)
{
    simplifier.max_resolution = 9999999;
    Polygon simplified = simplifier.polyline(sine);

    EXPECT_EQ(simplified.size(), 2) << "All zigzagged lines can be erased because they deviate less than the maximum deviation, leaving only the endpoints.";
}

}