//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include "../src/LayerPlan.h" //Code under test.

namespace cura
{

/*!
 * A fixture to test extruder plans with.
 *
 * It contains an extruder plan, optionally pre-filled with some paths.
 */
class ExtruderPlanTest : public testing::Test
{
public:
    /*!
     * One path with 5 vertices printing a 1000x1000 micron square starting from
     * 0,0.
     */
    std::vector<GCodePath> square;

    /*!
     * Three lines side by side, with two travel moves in between.
     */
    std::vector<GCodePath> lines;

    /*!
     * An extruder plan that can be used as a victim for testing.
     */
    ExtruderPlan extruder_plan;

    /*!
     * Configuration to print extruded paths with in the fixture.
     *
     * This config is referred to via pointer, so changing it will immediately
     * change it for all extruded paths.
     */
    GCodePathConfig extrusion_config;

    /*!
     * Configuration to print travel paths with in the fixture.
     *
     * This config is referred to via pointer, so changing it will immediately
     * change it for all travel paths.
     */
    GCodePathConfig travel_config;

    ExtruderPlanTest() :
        extruder_plan(
            /*extruder=*/0,
            /*layer_nr=*/50,
            /*is_initial_layer=*/false,
            /*is_raft_layer=*/false,
            /*layer_thickness=*/100,
            FanSpeedLayerTimeSettings(),
            RetractionConfig()
        ),
        extrusion_config(
            PrintFeatureType::OuterWall,
            /*line_width=*/400,
            extruder_plan.layer_thickness,
            /*flow=*/1.0_r,
            GCodePathConfig::SpeedDerivatives(50, 1000, 10)
        ),
        travel_config(
            PrintFeatureType::MoveCombing,
            /*line_width=*/0,
            extruder_plan.layer_thickness,
            /*flow=*/0.0_r,
            GCodePathConfig::SpeedDerivatives(120, 5000, 30)
        )
    {
        const std::string mesh_id = "test_mesh";
        constexpr Ratio flow_1 = 1.0_r;
        constexpr bool no_spiralize = false;
        constexpr Ratio speed_factor_1 = 1.0_r;
        square.assign({GCodePath(extrusion_config, mesh_id, SpaceFillType::PolyLines, flow_1, no_spiralize, speed_factor_1)});
        square.back().points = {
            Point(0, 0),
            Point(1000, 0),
            Point(1000, 1000),
            Point(0, 1000),
            Point(0, 0)
        };

        lines.assign({
            GCodePath(extrusion_config, mesh_id, SpaceFillType::Lines, flow_1, no_spiralize, speed_factor_1),
            GCodePath(travel_config, mesh_id, SpaceFillType::Lines, flow_1, no_spiralize, speed_factor_1),
            GCodePath(extrusion_config, mesh_id, SpaceFillType::Lines, flow_1, no_spiralize, speed_factor_1),
            GCodePath(travel_config, mesh_id, SpaceFillType::Lines, flow_1, no_spiralize, speed_factor_1),
            GCodePath(extrusion_config, mesh_id, SpaceFillType::Lines, flow_1, no_spiralize, speed_factor_1)
        });
        lines[0].points = {Point(0, 0), Point(1000, 0)};
        lines[1].points = {Point(1000, 0), Point(1000, 400)};
        lines[2].points = {Point(1000, 400), Point(0, 400)};
        lines[3].points = {Point(0, 400), Point(0, 800)};
        lines[4].points = {Point(0, 800), Point(1000, 800)};
    }
};

/*!
 * Tests that paths remain unmodified if applying back pressure compensation
 * with factor 0.
 */
TEST_F(ExtruderPlanTest, BackPressureCompensationZeroIsUncompensated)
{
    for(std::vector<GCodePath> path : {square, lines})
    {
        extruder_plan.paths = path;
        std::vector<Ratio> original_flows;
        for(const GCodePath& path : extruder_plan.paths)
        {
            original_flows.push_back(path.flow);
        }

        extruder_plan.applyBackPressureCompensation(0.0_r);

        ASSERT_EQ(extruder_plan.paths.size(), original_flows.size()) << "Number of paths may not have changed.";
        for(size_t i = 0; i < extruder_plan.paths.size(); ++i)
        {
            EXPECT_EQ(original_flows[i], extruder_plan.paths[i].flow) << "The flow rate did not change, because the back pressure compensation ratio was 0.";
        }
    }
}

}
