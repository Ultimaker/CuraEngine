//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include "../src/LayerPlan.h" //Code under test.

namespace cura
{

/*!
 * A fixture containing some sets of GCodePaths to test with.
 */
class ExtruderPlanTestPathCollection
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
     * Three lines side by side with travel moves in between, but adjusted flow.
     *
     * The first line gets 120% flow.
     * The second line gets 80% flow.
     * The third line gets 40% flow.
     */
    std::vector<GCodePath> decreasing_flow;

    /*!
     * Three lines side by side with their speed factors adjusted.
     *
     * The first line gets 120% speed.
     * The second line gets 80% speed.
     * The third line gets 40% speed.
     */
    std::vector<GCodePath> decreasing_speed;

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

    ExtruderPlanTestPathCollection() :
        extrusion_config(
            PrintFeatureType::OuterWall,
            /*line_width=*/400,
            /*layer_thickness=*/100,
            /*flow=*/1.0_r,
            GCodePathConfig::SpeedDerivatives(50, 1000, 10)
        ),
        travel_config(
            PrintFeatureType::MoveCombing,
            /*line_width=*/0,
            /*layer_thickness=*/100,
            /*flow=*/0.0_r,
            GCodePathConfig::SpeedDerivatives(120, 5000, 30)
        )
    {
        const std::string mesh_id = "test_mesh";
        constexpr Ratio flow_1 = 1.0_r;
        constexpr bool no_spiralize = false;
        constexpr Ratio speed_1 = 1.0_r;
        square.assign({GCodePath(extrusion_config, mesh_id, SpaceFillType::PolyLines, flow_1, no_spiralize, speed_1)});
        square.back().points = {
            Point(0, 0),
            Point(1000, 0),
            Point(1000, 1000),
            Point(0, 1000),
            Point(0, 0)
        };

        lines.assign({
            GCodePath(extrusion_config, mesh_id, SpaceFillType::Lines, flow_1, no_spiralize, speed_1),
            GCodePath(travel_config, mesh_id, SpaceFillType::Lines, flow_1, no_spiralize, speed_1),
            GCodePath(extrusion_config, mesh_id, SpaceFillType::Lines, flow_1, no_spiralize, speed_1),
            GCodePath(travel_config, mesh_id, SpaceFillType::Lines, flow_1, no_spiralize, speed_1),
            GCodePath(extrusion_config, mesh_id, SpaceFillType::Lines, flow_1, no_spiralize, speed_1)
        });
        lines[0].points = {Point(0, 0), Point(1000, 0)};
        lines[1].points = {Point(1000, 0), Point(1000, 400)};
        lines[2].points = {Point(1000, 400), Point(0, 400)};
        lines[3].points = {Point(0, 400), Point(0, 800)};
        lines[4].points = {Point(0, 800), Point(1000, 800)};

        constexpr Ratio flow_12 = 1.2_r;
        constexpr Ratio flow_08 = 0.8_r;
        constexpr Ratio flow_04 = 0.4_r;
        decreasing_flow.assign({
            GCodePath(extrusion_config, mesh_id, SpaceFillType::Lines, flow_12, no_spiralize, speed_1),
            GCodePath(travel_config, mesh_id, SpaceFillType::Lines, flow_1, no_spiralize, speed_1),
            GCodePath(extrusion_config, mesh_id, SpaceFillType::Lines, flow_08, no_spiralize, speed_1),
            GCodePath(travel_config, mesh_id, SpaceFillType::Lines, flow_1, no_spiralize, speed_1),
            GCodePath(extrusion_config, mesh_id, SpaceFillType::Lines, flow_04, no_spiralize, speed_1)
        });
        decreasing_flow[0].points = {Point(0, 0), Point(1000, 0)};
        decreasing_flow[1].points = {Point(1000, 0), Point(1000, 400)};
        decreasing_flow[2].points = {Point(1000, 400), Point(0, 400)};
        decreasing_flow[3].points = {Point(0, 400), Point(0, 800)};
        decreasing_flow[4].points = {Point(0, 800), Point(1000, 800)};

        constexpr Ratio speed_12 = 1.2_r;
        constexpr Ratio speed_08 = 0.8_r;
        constexpr Ratio speed_04 = 0.4_r;
        decreasing_speed.assign({
            GCodePath(extrusion_config, mesh_id, SpaceFillType::Lines, flow_1, no_spiralize, speed_12),
            GCodePath(travel_config, mesh_id, SpaceFillType::Lines, flow_1, no_spiralize, speed_1),
            GCodePath(extrusion_config, mesh_id, SpaceFillType::Lines, flow_1, no_spiralize, speed_08),
            GCodePath(travel_config, mesh_id, SpaceFillType::Lines, flow_1, no_spiralize, speed_1),
            GCodePath(extrusion_config, mesh_id, SpaceFillType::Lines, flow_1, no_spiralize, speed_04)
        });
        decreasing_speed[0].points = {Point(0, 0), Point(1000, 0)};
        decreasing_speed[1].points = {Point(1000, 0), Point(1000, 400)};
        decreasing_speed[2].points = {Point(1000, 400), Point(0, 400)};
        decreasing_speed[3].points = {Point(0, 400), Point(0, 800)};
        decreasing_speed[4].points = {Point(0, 800), Point(1000, 800)};
    }
};

/*!
 * Tests in this class get parameterized with a vector of GCodePaths to put in
 * the extruder plan, and an extruder plan to put it in.
 */
class ExtruderPlanPathsParameterizedTest : public testing::TestWithParam<std::vector<GCodePath>> {
public:
    /*!
     * An extruder plan that can be used as a victim for testing.
     */
    ExtruderPlan extruder_plan;

    ExtruderPlanPathsParameterizedTest() :
        extruder_plan(
            /*extruder=*/0,
            /*layer_nr=*/50,
            /*is_initial_layer=*/false,
            /*is_raft_layer=*/false,
            /*layer_thickness=*/100,
            FanSpeedLayerTimeSettings(),
            RetractionConfig()
        )
    {}
};

INSTANTIATE_TEST_CASE_P(ExtruderPlanTestInstantiation, ExtruderPlanPathsParameterizedTest, testing::Values(
        ExtruderPlanTestPathCollection().square,
        ExtruderPlanTestPathCollection().lines,
        ExtruderPlanTestPathCollection().decreasing_flow,
        ExtruderPlanTestPathCollection().decreasing_speed
));

/*!
 * Tests that paths remain unmodified if applying back pressure compensation
 * with factor 0.
 */
TEST_P(ExtruderPlanPathsParameterizedTest, BackPressureCompensationZeroIsUncompensated)
{
    extruder_plan.paths = GetParam();
    std::vector<Ratio> original_flows;
    std::vector<Ratio> original_speeds;
    for(const GCodePath& path : extruder_plan.paths)
    {
        original_flows.push_back(path.flow);
        original_speeds.push_back(path.speed_factor);
    }

    extruder_plan.applyBackPressureCompensation(0.0_r);

    ASSERT_EQ(extruder_plan.paths.size(), original_flows.size()) << "Number of paths may not have changed.";
    for(size_t i = 0; i < extruder_plan.paths.size(); ++i)
    {
        EXPECT_EQ(original_flows[i], extruder_plan.paths[i].flow) << "The flow rate did not change. Back pressure compensation doesn't adjust flow.";
        EXPECT_EQ(original_speeds[i], extruder_plan.paths[i].speed_factor) << "The speed factor did not change, since the compensation factor was 0.";
    }
}

}
