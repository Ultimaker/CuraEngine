//Copyright (c) 2019 Ultimaker B.V.
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
    Polygon test_square2; //Larger and more to the right.
    Polygon test_triangle;
    Polygon test_circle;
    Polygon test_convex_shape;
    Polygons test_shapes; //All above polygons! As well as an inset of 100 microns of them.

    PolygonConnector* pc;
    Polygons connecteds;

    virtual void SetUp()
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
        constexpr coord_t max_dist = 170;
        pc = new PolygonConnector(line_width, max_dist);
        pc->add(test_shapes);
        connecteds = pc->connect();

        ASSERT_GT(connecteds.size(), 0) << "PolygonConnector gave no output polygons!";
    }

    void TearDown()
    {
        delete pc;
    }

};

TEST_F(PolygonConnectorTest, getBridgeTest)
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

    std::optional<PolygonConnector::PolygonBridge> computed = pc->getBridge(test_square, polys);

    ASSERT_TRUE(bool(computed)) << "An answer is expected.";
    if (computed)
    {
        const coord_t a_from_error = vSize(predicted.a.from.p() - computed->a.from.p());
        const coord_t a_to_error = vSize(predicted.a.to.p() - computed->a.to.p());
        const coord_t b_from_error = vSize(predicted.b.from.p() - computed->b.from.p());
        const coord_t b_to_error = vSize(predicted.b.to.p() - computed->b.to.p());

        constexpr coord_t maximum_error = 10;
        EXPECT_LT(a_from_error, maximum_error) << "a.from was computed to something different from what was predicted!";
        EXPECT_LT(a_to_error, maximum_error) << "a.to was computed to something different from what was predicted!";
        EXPECT_LT(b_from_error, maximum_error) << "b.from was computed to something different from what was predicted!";
        EXPECT_LT(b_to_error, maximum_error) << "b.to was computed to something different from what was predicted!";
    }
}

TEST_F(PolygonConnectorTest, connectionLengthTest)
{
    constexpr coord_t maximum_distance = 170;
    std::unordered_set<Point> input_verts;
    for (ConstPolygonRef poly : test_shapes)
    {
        for (Point p : poly)
        {
            input_verts.emplace(p);
        }
    }

    coord_t longest_connection_dist = 0;
    size_t too_long_connection_count = 0;
    for (PolygonConnector::PolygonBridge bridge : pc->all_bridges)
    {
        for (auto connection : {bridge.a, bridge.b})
        {
            const coord_t connection_dist = vSize(connection.to.p() - connection.from.p());
            if (connection_dist > maximum_distance)
            {
                too_long_connection_count++;
                longest_connection_dist = std::max(longest_connection_dist, connection_dist);
            }
        }
    }

    ASSERT_EQ(too_long_connection_count, 0) << "PolygonConnector::connect() obtained " << too_long_connection_count << " too long bridge connections! Longest is " << INT2MM(longest_connection_dist) << "\n";
}


}
