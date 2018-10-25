//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PolygonConnectorTest.h"

#include <unordered_set>

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(PolygonConnectorTest);

void PolygonConnectorTest::setUp()
{
    
    test_square.emplace_back(0, 0);
    test_square.emplace_back(1000, 0);
    test_square.emplace_back(1000, 1000);
    test_square.emplace_back(0, 1000);
    test_shapes.add(test_square);
    
    test_square2.emplace_back(1100, 1500);
    test_square2.emplace_back(2000, 1500);
    test_square2.emplace_back(2000, -500);
    test_square2.emplace_back(1100, -500);
    test_shapes.add(test_square2);

    test_triangle.emplace_back(0, 2100);
    test_triangle.emplace_back(500, 1100);
    test_triangle.emplace_back(1500, 2100);
    test_shapes.add(test_triangle);
    
    for (double a = 0; a < 1.0; a += .05)
    {
        test_circle.add(Point(2050, 2050) + Point(std::cos(a * 2 * M_PI)*500, std::sin(a * 2 * M_PI)*500));
    }
    test_shapes.add(test_circle);

    test_convex_shape.emplace_back(-300, 0);
    test_convex_shape.emplace_back(-100, 500);
    test_convex_shape.emplace_back(-100, 600);
    test_convex_shape.emplace_back(-200, 1000);
    test_convex_shape.emplace_back(-500, 1500);
    test_convex_shape.emplace_back(-1500, 1500);
    test_convex_shape.emplace_back(-1500, 1500);
    test_convex_shape.emplace_back(-1600, 1100);
    test_convex_shape.emplace_back(-700, 200);
    test_shapes.add(test_convex_shape);

    Polygons inset = test_shapes;
    while (!inset.empty())
    {
        inset = inset.offset(-100);
        test_shapes.add(inset);
//         break;
    }

    line_width = 100;
    max_dist = 170;
    pc = new PolygonConnector(line_width, max_dist);
    pc->add(test_shapes);
    connecteds = pc->connect();
    CPPUNIT_ASSERT_MESSAGE("PolygonConnector gave no output polygons!!", connecteds.size() > 0);
}

void PolygonConnectorTest::tearDown()
{
    delete pc;
}

void PolygonConnectorTest::getBridgeTest()
{

    PolygonConnector::PolygonBridge predicted(
        PolygonConnector::PolygonConnection{
            ClosestPolygonPoint(Point(0, 500), 2, test_square),
            ClosestPolygonPoint(Point(-100, 500), 0, test_convex_shape)},
        PolygonConnector::PolygonConnection{
            ClosestPolygonPoint(Point(0, 600), 2, test_square),
            ClosestPolygonPoint(Point(-100, 600), 0, test_convex_shape)});
    std::vector<Polygon> polys;
    polys.push_back(test_convex_shape);
    getBridgeAssert(predicted, test_square, polys);
}

void PolygonConnectorTest::getBridgeAssert(std::optional<PolygonConnector::PolygonBridge> predicted, ConstPolygonRef from_poly, std::vector<Polygon>& to_polygons)
{
    std::optional<PolygonConnector::PolygonBridge> computed = pc->getBridge(from_poly, to_polygons);
    
    std::stringstream ss;
    ss << "PolygonConnector::getBridge(test_square, test_triangle) ";

    constexpr bool draw_problem_scenario = false; // make this true if you are debugging the function getNextParallelIntersection(.)

    if (draw_problem_scenario)
    {
        AABB aabb(from_poly);
        for (PolygonRef poly : to_polygons)
        {
            aabb.include(AABB(poly).min);
            aabb.include(AABB(poly).max);
        }
        SVG svg("output/bs.svg", aabb, Point(500, 500));
        svg.writePolygon(from_poly, SVG::Color::YELLOW, 4);
        for (PolygonRef poly : to_polygons)
        {
            svg.writePolygon(poly, SVG::Color::GRAY, 4);
        }
        if (computed)
        {
            svg.writeLine(computed->a.from.p(), computed->a.to.p(), SVG::Color::BLUE, 4);
            svg.writeLine(computed->b.from.p(), computed->b.to.p(), SVG::Color::GREEN, 4);
            Polygon connected = pc->connectPolygonsAlongBridge(*computed);
            svg.writePolygon(connected, SVG::Color::RED, 1);
//             svg.writePoints(connected, true, 5, SVG::Color::YELLOW);
//             int c = 0;
//             for (Point p : connected)
//                 svg.writeText(p, std::to_string(c++));
            std::cerr << "written\n";
        }
        else
        {
            std::cerr << "Couldn't find any connection!\n";
        }
        
    }
    
    if (predicted && computed)
    {
        coord_t a_from_error = vSize(predicted->a.from.p() - computed->a.from.p());
        coord_t a_to_error = vSize(predicted->a.to.p() - computed->a.to.p());
        coord_t b_from_error = vSize(predicted->b.from.p() - computed->b.from.p());
        coord_t b_to_error = vSize(predicted->b.to.p() - computed->b.to.p());
        ss << " computed to something different from what was predicted!\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), 
            a_from_error < maximum_error
            && a_to_error < maximum_error
            && b_from_error < maximum_error
            && b_to_error < maximum_error
        );
    }
    else if (predicted && !computed)
    {
        ss << " didn't give an answer, but it was predicted to give one!\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), false);
    }
    else if (!predicted && computed)
    {
        ss << " gave an answer, but it was predicted to not be possible!\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), false);
    }
}

void PolygonConnectorTest::connectionLengthTest()
{

    constexpr bool draw_problem_scenario = false; // make this true if you are debugging the function getNextParallelIntersection(.)

    std::unordered_set<Point> input_verts;
    for (ConstPolygonRef poly : test_shapes)
    {
        for (Point p : poly)
        {
            input_verts.emplace(p);
        }
    }
    if (draw_problem_scenario)
    {
        SVG svg("output/bs2.svg", AABB(test_shapes), Point(500, 500));
        svg.writePolygons(test_shapes, SVG::Color::YELLOW);
        svg.writePolygons(connecteds, SVG::Color::RED);
        for (PolygonConnector::PolygonBridge bridge : pc->all_bridges)
        {
            svg.writeLine(bridge.a.from.p(), bridge.a.to.p(), SVG::Color::BLUE);
            svg.writeLine(bridge.b.from.p(), bridge.b.to.p(), SVG::Color::GREEN);
        }
        std::cerr << "written\n";\
    }
    
    coord_t longest_connection_dist = 0;
    size_t too_long_connection_count = 0;
    for (PolygonConnector::PolygonBridge bridge : pc->all_bridges)
    {
        for (auto connection : {bridge.a, bridge.b})
        {
            coord_t connection_dist = vSize(connection.to.p() - connection.from.p());
            if (connection_dist > max_dist)
            {
                too_long_connection_count++;
                longest_connection_dist = std::max(longest_connection_dist, connection_dist);
            }
        }
    }

    std::stringstream ss;
    ss << "PolygonConnector::connect() obtained " << too_long_connection_count << " too long bridge connections! Longest is " << INT2MM(longest_connection_dist) << "\n";
    CPPUNIT_ASSERT_MESSAGE(ss.str(), too_long_connection_count == 0);
}


}
