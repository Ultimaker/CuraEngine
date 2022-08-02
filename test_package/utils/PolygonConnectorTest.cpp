// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/PolygonConnector.h" // The class under test.
#include "utils/Coord_t.h"
#include "utils/polygon.h" // To create polygons to test with.
#include <gtest/gtest.h>
#include <unordered_set>

// NOLINTBEGIN(*-magic-numbers)
namespace cura
{

// NOLINTBEGIN(misc-non-private-member-variables-in-classes)
class PolygonConnectorTest : public testing::Test
{
public:
    Polygon test_square;
    Polygon test_square_around; // Larger, around the first square.
    Polygon test_square_adjacent; // Next to the first square.
    Polygon test_triangle;
    Polygon test_circle;
    Polygon test_convex_shape;
    Polygons test_shapes; // All above polygons! As well as an inset of 100 microns of them.

    PolygonConnector* pc;
    Polygons connected_polygons;
    std::vector<VariableWidthLines> connected_paths;

    virtual void SetUp() override
    {
        test_square.emplace_back(0, 0);
        test_square.emplace_back(1000, 0);
        test_square.emplace_back(1000, 1000);
        test_square.emplace_back(0, 1000);

        test_square_around.emplace_back(1100, 1100);
        test_square_around.emplace_back(-100, 1100);
        test_square_around.emplace_back(-100, -100);
        test_square_around.emplace_back(1100, -100);

        test_square_adjacent.emplace_back(1100, 200);
        test_square_adjacent.emplace_back(2100, 200);
        test_square_adjacent.emplace_back(2100, 1200);
        test_square_adjacent.emplace_back(1100, 1200);

        constexpr coord_t line_width = 100;
        pc = new PolygonConnector(line_width);
    }

    void TearDown() override
    {
        delete pc;
    }
};
// NOLINTEND(misc-non-private-member-variables-in-classes)

/*!
 * Test creating a bridge between two squares that are nested in each other at
 * precisely the line width apart.
 *
 * This is a common occurrence with skin.
 */
TEST_F(PolygonConnectorTest, getBridgeNestedSquares)
{
    std::vector<Polygon> to_connect({ test_square_around });
    std::optional<PolygonConnector::PolygonBridge<Polygon>> bridge = pc->getBridge(test_square, to_connect);

    ASSERT_NE(bridge, std::nullopt) << "The two polygons are nested simply, so they are definitely positioned closely enough to bridge. They are also wide enough.";

    EXPECT_EQ(vSize(bridge->a.from_point - bridge->a.to_point), 100) << "The polygons are 100 units spaced out concentrically, so this is the shortest possible bridge.";
    EXPECT_EQ(vSize(bridge->b.from_point - bridge->b.to_point), 100) << "The second bridge should also be equally short in this case.";
    EXPECT_EQ(LinearAlg2D::getDist2BetweenLineSegments(bridge->a.from_point, bridge->a.to_point, bridge->b.from_point, bridge->b.to_point), 100 * 100) << "The bridges should be spaced 1 line width (100 units) apart.";
    EXPECT_LT(LinearAlg2D::pointIsLeftOfLine(bridge->b.from_point, bridge->a.from_point, bridge->a.to_point), 0) << "Connection B should be to the right of connection A.";
    EXPECT_LT(LinearAlg2D::pointIsLeftOfLine(bridge->b.to_point, bridge->a.from_point, bridge->a.to_point), 0) << "Connection B should be to the right of connection A.";
}

/*!
 * Test creating a bridge between two adjacent squares that are spaced 1 line
 * width apart.
 *
 * This is a common occurrence with multiplied infill.
 */
TEST_F(PolygonConnectorTest, getBridgeAdjacentSquares)
{
    std::vector<Polygon> to_connect({ test_square_adjacent });
    std::optional<PolygonConnector::PolygonBridge<Polygon>> bridge = pc->getBridge(test_square, to_connect);

    ASSERT_NE(bridge, std::nullopt) << "The two polygons are adjacent, spaced closely enough to bridge and with enough room.";

    EXPECT_EQ(vSize(bridge->a.from_point - bridge->a.to_point), 100) << "The polygons are 100 units spaced apart, so this is the shortest possible bridge.";
    EXPECT_EQ(vSize(bridge->b.from_point - bridge->b.to_point), 100) << "The second bridge should also be equally short in this case.";
    EXPECT_EQ(LinearAlg2D::getDist2BetweenLineSegments(bridge->a.from_point, bridge->a.to_point, bridge->b.from_point, bridge->b.to_point), 100 * 100) << "The bridges should be spaced 1 line width (100 units) apart.";
    EXPECT_LT(LinearAlg2D::pointIsLeftOfLine(bridge->b.from_point, bridge->a.from_point, bridge->a.to_point), 0) << "Connection B should be to the right of connection A.";
    EXPECT_LT(LinearAlg2D::pointIsLeftOfLine(bridge->b.to_point, bridge->a.from_point, bridge->a.to_point), 0) << "Connection B should be to the right of connection A.";
}

/*!
 * Test that the bridge is created in the location where the polygons are
 * closest together.
 */
TEST_F(PolygonConnectorTest, getBridgeClosest)
{
    Polygon adjacent_slanted; // A polygon that's adjacent to the first square, but tilted such that the vertex at [1100,200] is definitely
                              // the closest.
    adjacent_slanted.emplace_back(1100, 200);
    adjacent_slanted.emplace_back(2100, 200);
    adjacent_slanted.emplace_back(2140, 1200);
    adjacent_slanted.emplace_back(1140, 1200);
    std::vector<Polygon> to_connect({ adjacent_slanted });

    std::optional<PolygonConnector::PolygonBridge<Polygon>> bridge = pc->getBridge(test_square, to_connect);

    ASSERT_NE(bridge, std::nullopt) << "The two polygons are adjacent and spaced closely enough to bridge along their entire side, even with the slant.";

    EXPECT_EQ(bridge->b.from_point, Point(1000, 200)) << "The closest connection is [1000,200] -> [1100,200]. There is no space to the right of that, so bridge B should be there.";
    EXPECT_EQ(bridge->b.to_point, Point(1100, 200)) << "The closest connection is [1000,200] -> [1100,200]. There is no space to the right of that, so bridge B should be there.";
    EXPECT_GT(LinearAlg2D::pointIsLeftOfLine(bridge->a.from_point, bridge->b.from_point, bridge->b.to_point), 0) << "Connection A should be to the left of connection B.";
    EXPECT_GT(LinearAlg2D::pointIsLeftOfLine(bridge->a.to_point, bridge->b.from_point, bridge->b.to_point), 0) << "Connection A should be to the left of connection B.";
}

/*!
 * Test attempting to create a bridge when the polygons are too far apart.
 */
TEST_F(PolygonConnectorTest, getBridgeTooFar)
{
    Polygon too_far; // More than 1.5 line width away.
    too_far.emplace_back(1200, 0);
    too_far.emplace_back(2200, 0);
    too_far.emplace_back(2200, 1000);
    too_far.emplace_back(1200, 1000);
    std::vector<Polygon> to_connect({ too_far });

    std::optional<PolygonConnector::PolygonBridge<Polygon>> bridge = pc->getBridge(test_square, to_connect);

    EXPECT_EQ(bridge, std::nullopt) << "The two polygons are 200 units apart where they are closest, which is more than 1.5 times the line width (100), so they can't be connected.";
}

/*!
 * Test attempting to create a bridge when the connecting part is too narrow.
 *
 * Since the bridging lines need to be spaced 1 line width apart, they can't be
 * too close together. If the bridge can't be constructed keeping proper spacing
 * the bridge should fail to be created.
 */
TEST_F(PolygonConnectorTest, getBridgeTooNarrow)
{
    Polygon too_narrow;
    too_narrow.emplace_back(1100, 400);
    too_narrow.emplace_back(2100, 400);
    too_narrow.emplace_back(2100, 480); // Less than 100 units wide.
    too_narrow.emplace_back(1100, 480);
    std::vector<Polygon> to_connect({ too_narrow });

    std::optional<PolygonConnector::PolygonBridge<Polygon>> bridge = pc->getBridge(test_square, to_connect);

    EXPECT_EQ(bridge, std::nullopt) << "Where the two polygons are adjacent is only 80 units wide. This is not enough to create a bridge with the connecting lines spaced 1 line width (100 units) apart.";
}

/*!
 * Try connecting four nested polygons.
 *
 * Let's play a game of connect four!
 */
TEST_F(PolygonConnectorTest, connectFourNested)
{
    Polygons connecting;
    connecting.add(test_square_around); // 1200-wide square.
    connecting.add(test_square); // 1000-wide square.
    connecting.add(test_square.offset(-100)); // 800-wide square.
    connecting.add(test_square.offset(-200)); // 600-wide square.

    pc->add(connecting);
    Polygons output_polygons;
    std::vector<VariableWidthLines> output_paths;
    pc->connect(output_polygons, output_paths);

    EXPECT_EQ(output_polygons.size(), 1) << "All four polygons should've gotten connected into 1 single polygon.";
}
} // namespace cura
// NOLINTEND(*-magic-numbers)
