//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>
#include <unordered_set>

#include <../src/utils/polygon.h> //To create polygons to test with.
#include <../src/utils/PolygonConnector.h> //The class under test.

namespace cura
{

class PolygonConnectorTest : public testing::Test
{
public:
    Polygon test_square;
    Polygon test_square2; //Larger, around the first square.
    Polygon test_triangle;
    Polygon test_circle;
    Polygon test_convex_shape;
    Polygons test_shapes; //All above polygons! As well as an inset of 100 microns of them.

    PolygonConnector* pc;
    Polygons connected_polygons;
    VariableWidthPaths connected_paths;

    virtual void SetUp()
    {
        test_square.emplace_back(0, 0);
        test_square.emplace_back(1000, 0);
        test_square.emplace_back(1000, 1000);
        test_square.emplace_back(0, 1000);
        test_shapes.add(test_square);
    
        test_square2.emplace_back(1100, 1100);
        test_square2.emplace_back(-100, 1100);
        test_square2.emplace_back(-100, -100);
        test_square2.emplace_back(1100, -100);
        test_shapes.add(test_square2);

        test_triangle.emplace_back(0, 2100);
        test_triangle.emplace_back(500, 1100);
        test_triangle.emplace_back(1500, 2100);
        test_shapes.add(test_triangle);

        for (double a = 0; a < 1.0; a += 0.05)
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
        }

        constexpr coord_t line_width = 100;
        pc = new PolygonConnector(line_width);
        pc->add(test_shapes);
        pc->connect(connected_polygons, connected_paths);

        ASSERT_GT(connected_polygons.size(), 0) << "PolygonConnector gave no output polygons!";
    }

    void TearDown()
    {
        delete pc;
    }

};

/*!
 * Test creating a bridge between two squares that are nested next to each other
 * at precisely the line width apart.
 *
 * This is a common occurrence with skin.
 */
TEST_F(PolygonConnectorTest, getBridgeNestedSquares)
{
    std::vector<Polygon> to_connect({test_square2});
    std::optional<PolygonConnector::PolygonBridge<Polygon>> bridge = pc->getBridge(test_square, to_connect);

    ASSERT_NE(bridge, std::nullopt) << "The two polygons are nested simply, so they are definitely positioned closely enough to bridge. They are also wide enough.";

    EXPECT_EQ(vSize(bridge->a.from_point - bridge->a.to_point), 100) << "The polygons are 100 units spaced out concentrically, so this is the shortest possible bridge.";
    EXPECT_EQ(vSize(bridge->b.from_point - bridge->b.to_point), 100) << "The second bridge should also be equally short in this case.";
    EXPECT_EQ(LinearAlg2D::getDist2BetweenLineSegments(bridge->a.from_point, bridge->a.to_point, bridge->b.from_point, bridge->b.to_point), 100 * 100) << "The bridges should be spaced 1 line width (100 units) apart.";
    EXPECT_LT(LinearAlg2D::pointIsLeftOfLine(bridge->b.from_point, bridge->a.from_point, bridge->a.to_point), 0) << "Connection B should be to the right of connection A.";
    EXPECT_LT(LinearAlg2D::pointIsLeftOfLine(bridge->b.to_point, bridge->a.from_point, bridge->a.to_point), 0) << "Connection B should be to the right of connection A.";
}

}
