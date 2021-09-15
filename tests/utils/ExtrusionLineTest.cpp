//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <../src/utils/ExtrusionLine.h> //The class under test.
#include <../src/utils/linearAlg2D.h>

#include <limits>

namespace cura
{
    TEST(ExtrusionLineTest, simplifyCircle)
    {
        ExtrusionLine  circle_polylines(0, false);
        auto& circle = circle_polylines.junctions;
        constexpr coord_t radius = 100000;
        constexpr double segment_length = 1000;
        constexpr double tau = 6.283185307179586476925286766559; //2 * pi.
        constexpr double increment = segment_length / radius; //Segments of 1000 units.
        for (double angle = 0; angle < tau; angle += increment)
        {
            circle.emplace_back(Point(std::cos(angle) * radius, std::sin(angle) * radius), 500, 0);
        }

        constexpr coord_t minimum_segment_length = segment_length + 10;
        circle_polylines.simplify(minimum_segment_length * minimum_segment_length, std::numeric_limits<coord_t>::max(), 2000); //With segments of 1000, we need to remove exactly half of the vertices to meet the requirement that all segments are >1010.
        constexpr coord_t maximum_segment_length = segment_length * 2 + 20; //+20 for some error margin due to rounding.

        for (size_t point_index = 1; point_index + 1 < circle.size(); point_index++) //Don't check the last vertex. Due to odd-numbered vertices it has to be shorter than the minimum.
        {
            coord_t segment_length = vSize(circle[point_index % circle.size()].p - circle[point_index - 1].p);
            ASSERT_GE(segment_length, minimum_segment_length) << "Segment " << (point_index - 1) << " - " << point_index << " is too short!";
            ASSERT_LE(segment_length, maximum_segment_length) << "Segment " << (point_index - 1) << " - " << point_index << " is too long!";
        }
    }

    TEST(ExtrusionLineTest, simplifyZigzag)
    {
        //Tests a zigzag line: /\/\/\/\/\/\/
        //If all line segments are short, they can all be removed and turned into one long line: -------------------
        ExtrusionLine zigzag_polylines(0, false);
        auto& zigzag = zigzag_polylines.junctions;
        constexpr coord_t segment_length = 1000;
        const coord_t y_increment = segment_length / std::sqrt(2);
        coord_t x = y_increment / 2;
        size_t zigzag_count = 100;

        for (size_t i = 0; i < zigzag_count; i++)
        {
            zigzag.emplace_back(Point(x, i * y_increment), 500, 0);
            x = 0 - x;
        }

        // we make the smallest_line_segment just smaller than the line from the start to the second to last point
        // so that we ensure the polyline does not get removed altogether
        const coord_t smallest_line_dist = (y_increment * (zigzag_count - 2) - 100);
        zigzag_polylines.simplify(smallest_line_dist * smallest_line_dist, y_increment * y_increment, 2000);

        EXPECT_THAT(zigzag.size(), testing::AllOf(testing::Ge(3), testing::Le(5))) << "All but the last zigzag should be removed.";
    }

    TEST(ExtrusionLineTest, simplifyLimitedLength)
    {
        //Generate a spiral with segments that gradually increase in length.
        ExtrusionLine spiral_polylines(0, false);
        auto& spiral = spiral_polylines.junctions;
        spiral.emplace_back(Point(), 500, 0);

        coord_t segment_length = 1000;
        double angle = 0;
        Point last_position;

        for (size_t i = 0; i < 10; i++)
        {
            const coord_t dx = std::cos(angle) * segment_length;
            const coord_t dy = std::sin(angle) * segment_length;
            last_position += Point(dx, dy);
            spiral.emplace_back(last_position, 500, 0);
            segment_length += 100;
            angle += 0.1;
        }

        spiral_polylines.simplify(1550 * 1550, std::numeric_limits<coord_t>::max(), 2000); //Remove segments smaller than 1550 (infinite area error).

        ASSERT_EQ(spiral.size(), 11 - 3) << "Should merge segments of length 1100 with 1200, 1300 with 1400 and first with last.";
    }

    TEST(ExtrusionLineTest, simplifyLimitedError)
    {
        // Generate a large polyline with a small region with arbitrary simplifiable geometry
        // We choose a spiral for that arbitrary geometry

        //Generate a square spiral with increasingly large corners
        ExtrusionLine spiral_polylines(0, false);
        auto& spiral = spiral_polylines.junctions;
        spiral.emplace_back(Point(-15000, 11000), 500, 0); // introduce points that may never be simplified so that we know the simplification should start at 0,0
        spiral.emplace_back(Point(-15000, 1000), 500, 0);
        spiral.emplace_back(Point(0, 0), 0.5 ,0);

        //Generate a square spiral, 90 degree corners to make it easy to compute the area loss while retaining a positive area per corner.
        coord_t segment_length = 1000;
        Point last_position(0, 0);
        double angle = 0;
        for (size_t i = 0; i < 10; i++)
        {
            const coord_t dx = std::cos(angle) * segment_length;
            const coord_t dy = std::sin(angle) * segment_length;
            last_position += Point(dx, dy);
            spiral.emplace_back(last_position, 500, 0);
            segment_length += 100;
            angle += M_PI / 2;
        }
        auto spiral_before = spiral; //copy

        coord_t max_height = segment_length * std::sqrt(2.0); // the diameter of the circle along the diagonal

        for (size_t i = 0; i < spiral_before.size(); ++i)
        {
            // apply simplify iteratively for each point until nothing is simplifiable any more
            spiral_polylines.simplify(10000 * 10000, max_height * max_height, 2000);
        }

        EXPECT_THAT(spiral.size(), testing::Eq(4)) << "Should simplify all spiral points except those connected to far away geometry.";
    }


    TEST(ExtrusionLineTest, simplifyIncreasingLimitedError)
    {
        // Generate a zigzag with bends in the outer ends with increasing height
        //   _   /\                        .
        //  | | | |
        //  | | | |
        //  | | | |
        //  | | | |
        //     v   \/

        ExtrusionLine zigzag_polylines(0, false);
        auto& zigzag = zigzag_polylines.junctions;

        constexpr coord_t non_simplifiable_bound = 10000;
        constexpr coord_t max_height = 562;

        coord_t long_segment = non_simplifiable_bound + 100;

        constexpr coord_t width = 1000;

        zigzag.emplace_back(Point(0, 0), 500, 0);

        size_t simplifiable_bend_count = 0;

        for (coord_t height = 100; height < 1000 || (height == 1000 && long_segment < 0); height += 25)
        {
            Point last_position = zigzag.back().p;
            zigzag.emplace_back(last_position + Point(width / 2, height), 500, 0);
            zigzag.emplace_back(last_position + Point(width, 0), 500, 0);
            zigzag.emplace_back(last_position + Point(width, long_segment), 500, 0);
            long_segment *= -1;
            if (height < max_height)
            {
                simplifiable_bend_count++;
            }
        }
        zigzag.emplace_back(zigzag.back().p + Point(-non_simplifiable_bound, -non_simplifiable_bound), 0.5 ,0); // complete polyline with a point which enforces non-simplification to both its segments

        auto zigzag_before = zigzag; //copy

        zigzag_polylines.simplify(non_simplifiable_bound * non_simplifiable_bound, max_height * max_height, 2000);

        EXPECT_THAT(zigzag.size(), testing::Eq(zigzag_before.size() - simplifiable_bend_count)) << "Should simplify bends with height 100 up to 500";
    }


    TEST(ExtrusionLineTest, simplifySineLimitedError)
    {
        // Generate a straight line with sinusoidal errors which should be simplified back into a more straight line

        // Hypothetically simplify() might replace each half period of the sine with a straight segment,
        // but because simplify() is heuristic it introduces more segments.
        // The function signature doesn't provide any guarantee about how much simplification will occur,
        // but in practice it should at least simplify up to double the minimal segment count.

        ExtrusionLine sine_polylines(0, false);
        auto& sine = sine_polylines.junctions;

        constexpr coord_t length = 10000;
        constexpr coord_t deviation = 500;
        constexpr size_t bulge_count = 5;

        sine.emplace_back(Point(length, 0), 500, 0);
        sine.emplace_back(Point(length, length), 500, 0);
        sine.emplace_back(Point(0, length), 500, 0);
        sine.emplace_back(Point(0, 0), 500, 0);

        for (coord_t x = 100; x < length; x += 100)
        {
            sine.emplace_back(Point(x, std::sin(INT2MM(x) / INT2MM(length) * M_PI * bulge_count) * deviation), 500, 0);
        }

        sine_polylines.simplify((length * length) / 4, 4 * deviation * deviation, 2000);

        const size_t max_simplified_sine_segments = bulge_count * 2; // * 2 because simplify() is not precise and might not optimally simplify
        EXPECT_THAT(sine.size(), testing::AllOf(testing::Ge(4), testing::Le(4 + max_simplified_sine_segments))) << "Should simplify each outward and each inward bulge.";
    }

    TEST(ExtrusionLineTest, simplifySineHighPoly)
    {
        // Generate a straight line with sinusoidal errors which should be simplified back into a more straight line

        // Hypothetically simplify() might replace each half period of the sine with a straight segment,
        // but because simplify() is heuristic it introduces more segments.
        // The function signature doesn't provide any guarantee about how much simplification will occur,
        // but in practice it should at least simplify up to double the minimal segment count.

        ExtrusionLine sine_polylines(0, false);
        auto& sine = sine_polylines.junctions;

        constexpr coord_t length = 1000;
        constexpr coord_t deviation = 100;
        constexpr size_t bulge_count = 17;
        constexpr coord_t allowed_simplification_height = 30;
        constexpr coord_t allowed_segment_length = 100;

        sine.emplace_back(Point(length, 0), 500, 0);
        sine.emplace_back(Point(length, length), 500, 0);
        sine.emplace_back(Point(0, length), 500, 0);
        sine.emplace_back(Point(0, 0), 500, 0);

        for (coord_t x = 1; x < length; x += 1)
        {
            coord_t y = std::sin(INT2MM(x) / INT2MM(length) * M_PI * bulge_count) * deviation;
            if (!sine.empty())
            {
                coord_t y_mid = (y + sine.back().p.Y) / 2;
                if (y_mid != y && y_mid != sine.back().p.Y
                    && sine.back().p.X == x - 1
                    )
                {
                    sine.emplace_back(Point(x, y_mid), 500, 0);
                }
            }
            sine.emplace_back(Point(x, y), 500, 0);
            if (x % (allowed_segment_length * 3) == 0) x += allowed_segment_length + 5;
        }
        auto sine_before = sine; //copy

        sine_polylines.simplify(allowed_segment_length * allowed_segment_length, allowed_simplification_height * allowed_simplification_height, 2000);

        // find largest height deviation
        coord_t largest_dist = 0;
        for (const ExtrusionJunction& from : sine_before)
        {
            coord_t closest_dist = std::numeric_limits<coord_t>::max();
            for (size_t i = 1; i < sine.size(); ++i)
            {
                const coord_t current_closest = vSize(cura::LinearAlg2D::getClosestOnLineSegment(from.p, sine[i - 1].p, sine[i].p) - from.p);
                closest_dist = std::min(closest_dist, current_closest);
            }
            largest_dist = std::max(largest_dist, closest_dist);
        }

        EXPECT_THAT(largest_dist, testing::Le(allowed_simplification_height + 10)) << "Shouldn't exceed maximum error distance";
    }

    TEST(ExtrusionLineTest, simplifyCircleLimitedError)
    {
        //Generate a circle with increasing resolution
        ExtrusionLine circle_polylines(0, false);
        auto& circle = circle_polylines.junctions;

        coord_t radius = 20000;
        coord_t segment_length = 100;
        for (double angle = 0; angle < 2 * M_PI; )
        {
            const coord_t dx = std::cos(angle) * radius;
            const coord_t dy = std::sin(angle) * radius;
            Point new_point(dx, dy);
            //assert(circle.empty() || std::abs(vSize(circle.back() - new_point) - segment_length) < 10); // we should now add a segment of the prescribed length
            circle.emplace_back(new_point, 500, 0);
            segment_length += 100;
            angle += 2.0 * std::asin(0.5 * INT2MM(segment_length) / INT2MM(radius));
        }

        auto circle_before = circle; //copy

        coord_t allowed_simplification_height = 1000;

        circle_polylines.simplify(std::numeric_limits<coord_t>::max(), allowed_simplification_height * allowed_simplification_height, 2000);

        // find largest height deviation
        coord_t largest_dist = 0;
        for (const ExtrusionJunction& from : circle_before)
        {
            coord_t closest_dist = std::numeric_limits<coord_t>::max();
            for (size_t i = 1; i < circle.size(); ++i)
            {
                const coord_t current_closest = vSize(cura::LinearAlg2D::getClosestOnLineSegment(from.p, circle[i - 1].p, circle[i].p) - from.p);
                closest_dist = std::min(closest_dist, current_closest);
            }
            largest_dist = std::max(largest_dist, closest_dist);
        }

        EXPECT_THAT(largest_dist, testing::Le(allowed_simplification_height + 10)) << "Shouldn't exceed maximum error distance";
    }

    TEST(ExtrusionLineTest, simplifyCircleHighPoly)
    {
        //Generate a circle with extremely high point count, such that all segments are within rounding distance 
        ExtrusionLine circle_polylines(0, false);
        auto& circle = circle_polylines.junctions;

        coord_t radius = 2000;
        coord_t segment_length = 1;
        coord_t allowed_simplification_height = 50;

        for (double angle = 0; angle < 2 * M_PI; )
        {
            const coord_t x = std::cos(angle) * radius;
            const coord_t y = std::sin(angle) * radius;
            Point new_point(x, y);
            circle.emplace_back(new_point, 500, 0);
            coord_t segment_length_here = segment_length;
            if ((x + y) % 5 == 0)
            {
                segment_length_here = 500;
            }
            angle += 2.0 * std::asin(0.5 * INT2MM(segment_length_here) / INT2MM(radius));
        }

        auto circle_before = circle; //copy

        circle_polylines.simplify(9999999, allowed_simplification_height * allowed_simplification_height, 2000);

        // find largest height deviation
        coord_t largest_dist = 0;
        for (const ExtrusionJunction& from : circle_before)
        {
            coord_t closest_dist = std::numeric_limits<coord_t>::max();
            for (size_t i = 1; i < circle.size(); ++i)
            {
                const coord_t current_closest = vSize(cura::LinearAlg2D::getClosestOnLineSegment(from.p, circle[i - 1].p, circle[i].p) - from.p);
                closest_dist = std::min(closest_dist, current_closest);
            }
            largest_dist = std::max(largest_dist, closest_dist);
        }

        EXPECT_THAT(largest_dist, testing::Le(allowed_simplification_height + 5)) << "Shouldn't exceed maximum error distance";
    }

    TEST(ExtrusionLineTest, simplifyColinear)
    {
        //Generate a line with several vertices halfway.
        constexpr coord_t spacing = 100;
        ExtrusionLine colinear_polylines(0, false);;
        auto& colinear = colinear_polylines.junctions;
        for (size_t i = 0; i < 10; i++)
        {
            colinear.emplace_back(Point(i * spacing + i % 2 - 1, i * spacing + i % 2 - 1), 500, 0); //Some jitter of 2 microns is allowed.
        }
        colinear.emplace_back(Point(spacing * 9, 0), 500, 0); //Make it a triangle so that the area is not 0 or anything.

        colinear_polylines.simplify(20 * 20, 20 * 20, 2000); //Regardless of parameters, it should always remove vertices with less than 5 micron deviation.

        ASSERT_GE(colinear_polylines.junctions.size(), 1) << "polyline shouldn't have gotten removed altogether";
        ASSERT_LE(colinear_polylines.junctions.size(), 8) << "At least half of the colinear points should have been removed";
    }

    TEST(ExtrusionLineTest, simplifyDegenerateVertex)
    {
        //Generate a line with several vertices halfway.
        constexpr coord_t spacing = 100;
        ExtrusionLine colinear_polylines(0, false);;
        auto& colinear = colinear_polylines.junctions;
        colinear.emplace_back(Point(0, 0), 500, 0);
        colinear.emplace_back(Point(0, 0), 500, 0); // degenerate vertex (same as begin)
        colinear.emplace_back(Point(spacing, spacing), 500, 0); // degenerate vertex (same as end)
        colinear.emplace_back(Point(spacing, spacing), 500, 0);

        colinear_polylines.simplify(20 * 20, 5 * 5, std::numeric_limits<coord_t>::max()); //Regardless of parameters, it should always remove those middle vertices

        ASSERT_EQ(colinear_polylines.junctions.size(), 2) << "The degenerate vertices should have been removed.";
    }

    /*
     * Test whether a polyline can be reduced to 1 or 2 vertices. In that case, it
     * should get reduced to 0 or stay at 3.
     */
    TEST(ExtrusionLineTest, simplifyToDegenerate)
    {
        //Generate a D shape with one long side and another curved side broken up into smaller pieces that can be removed.
        ExtrusionLine d_polylines(0, false);
        auto& d = d_polylines.junctions;
        d.emplace_back(Point(0, 0), 500, 0);
        d.emplace_back(Point(10, 55), 500, 0);
        d.emplace_back(Point(0, 110), 500, 0);

        d_polylines.simplify(100 * 100, 15 * 15, 2000);

        EXPECT_NE(d.size(), 1);
        EXPECT_NE(d.size(), 2);
    }

    TEST(ExtrusionLineTest, simplifyLineWidthVariance)
    {
        //Generate a line with several vertices halfway.
        constexpr coord_t spacing = 100;
        ExtrusionLine colinear_polylines(0, false);;
        auto& colinear = colinear_polylines.junctions;
        colinear.emplace_back(Point(0, 0), 200, 0);
        colinear.emplace_back(Point(spacing / 4, 0), 200, 0);
        colinear.emplace_back(Point(spacing / 2, 0), 400, 0);
        colinear.emplace_back(Point(spacing / 2 + spacing / 4, 0), 600, 0);
        colinear.emplace_back(Point(spacing, 0), 800, 0);

        colinear_polylines.simplify(25 * 25, 25 * 25, std::numeric_limits<coord_t>::max());

        ASSERT_EQ(colinear_polylines.junctions.size(), 2) << "The degenerate vertices should have been removed.";
        ASSERT_EQ(colinear[1].w, 500) << "The width of the end-junction should be the average of all removed."; // Since the distances where equal, and they should all have been merged with that one.
    }

    TEST(ExtrusionLineTest, simplifyNoLineWidthVariance)
    {
        //Generate a line with several vertices halfway.
        constexpr coord_t spacing = 100;
        ExtrusionLine colinear_polylines(0, false);;
        auto& colinear = colinear_polylines.junctions;
        colinear.emplace_back(Point(0, 0), 200, 0);
        colinear.emplace_back(Point(spacing / 4, 0), 200, 0);
        colinear.emplace_back(Point(spacing / 2, 0), 400, 0);
        colinear.emplace_back(Point(spacing / 2 + spacing / 4, 0), 600, 0);
        colinear.emplace_back(Point(spacing, 0), 800, 0);

        colinear_polylines.simplify(25 * 25, 25 * 25, 1);

        ASSERT_EQ(colinear_polylines.junctions.size(), 5) << "No junctions should have been removed."; // ... even though they are co-linear!
        ASSERT_EQ(colinear.back().w, 800) << "The width of the end-junction should not have been changed.";
    }
}
