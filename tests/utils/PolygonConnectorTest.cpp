//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PolygonConnectorTest.h"


namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(PolygonConnectorTest);

void PolygonConnectorTest::setUp()
{
    test_square.emplace_back(0, 0);
    test_square.emplace_back(100, 0);
    test_square.emplace_back(100, 100);
    test_square.emplace_back(0, 100);
    test_shapes.add(test_square);

    test_triangle.emplace_back(50, 110);
    test_triangle.emplace_back(150, 210);
    test_triangle.emplace_back(0, 210);
    test_shapes.add(test_triangle);

}

void PolygonConnectorTest::tearDown()
{
    //Do nothing.
}

void PolygonConnectorTest::getBridgeTest()
{
    std::vector<ConstPolygonPointer> polys;
    polys.push_back(test_triangle);
    polys.push_back(test_square);
    PolygonConnector::PolygonBridge predicted;
    predicted.a.from = ClosestPolygonPoint(Point(50,100), 2, test_square);
    predicted.a.to = ClosestPolygonPoint(Point(50, 110), 0, test_triangle);
    predicted.b.from = ClosestPolygonPoint(Point(60,100), 2, test_square);
    predicted.b.to = ClosestPolygonPoint(Point(60, 120), 0, test_triangle);
    getBridgeAssert(predicted, test_square, polys);
}

void PolygonConnectorTest::getBridgeAssert(std::optional<PolygonConnector::PolygonBridge> predicted, ConstPolygonRef from_poly, std::vector<ConstPolygonPointer>& to_polygons)
{
    const coord_t line_width = 10;
    const coord_t max_dist = 15;
    PolygonConnector pc(line_width, max_dist);
    pc.add(test_shapes);

    std::optional<PolygonConnector::PolygonBridge> computed = pc.getBridge(from_poly, to_polygons);
    
    std::stringstream ss;
    ss << "PolygonUtils::getBridge(test_square, test_triangle) ";

    constexpr bool draw_problem_scenario = true; // make this true if you are debugging the function getNextParallelIntersection(.)

    if (draw_problem_scenario)
    {
        SVG svg("output/bs.svg", AABB(test_shapes), Point(500,500));
        svg.writePolygons(test_shapes);
        if (computed)
        {
            svg.writeLine(computed->a.from.p(), computed->a.to.p(), SVG::Color::BLUE);
            svg.writeLine(computed->b.from.p(), computed->b.to.p(), SVG::Color::GREEN);
            Polygon connected = pc.connect(*computed);
            svg.writePolygon(connected, SVG::Color::YELLOW);
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
}

}
