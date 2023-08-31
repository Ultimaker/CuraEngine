// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "LayerPlan.h" //Code under test.
#include "pathPlanning/SpeedDerivatives.h"

#include <gtest/gtest.h>
#include <numeric> //For calculating averages.

// NOLINTBEGIN(*-magic-numbers)
namespace cura
{
/*!
 * A fixture containing some sets of GCodePaths to test with.
 */
// NOLINTBEGIN(misc-non-private-member-variables-in-classes)
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
     * A series of paths with variable line width.
     *
     * This one has no travel moves in between.
     * The last path gets a width of 0.
     */
    std::vector<GCodePath> variable_width;

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

    ExtruderPlanTestPathCollection()
        : extrusion_config(GCodePathConfig{ .type = PrintFeatureType::OuterWall, .line_width = 400, .layer_thickness = 100, .flow = 1.0_r, .speed_derivatives = SpeedDerivatives { .speed = 50.0, .acceleration = 1000.0, .jerk = 10.0 } })
        , travel_config(GCodePathConfig{ .type = PrintFeatureType::MoveCombing, .line_width = 0, .layer_thickness = 100, .flow = 0.0_r, .speed_derivatives = SpeedDerivatives { .speed = 120.0, .acceleration = 5000.0, .jerk = 30.0 } })
    {
        std::shared_ptr<SliceMeshStorage> mesh = nullptr;
        constexpr Ratio flow_1 = 1.0_r;
        constexpr Ratio width_1 = 1.0_r;
        constexpr bool no_spiralize = false;
        constexpr Ratio speed_1 = 1.0_r;
        square.assign({ GCodePath{ .config = extrusion_config,
                                   .mesh = mesh,
                                   .space_fill_type = SpaceFillType::PolyLines,
                                   .flow = flow_1,
                                   .width_factor = width_1,
                                   .spiralize = no_spiralize,
                                   .speed_factor = speed_1 } });

        square.back().points = { Point(0, 0), Point(1000, 0), Point(1000, 1000), Point(0, 1000), Point(0, 0) };

        lines.assign({ GCodePath{ .config = extrusion_config,
                                  .mesh = mesh,
                                  .space_fill_type = SpaceFillType::Lines,
                                  .flow = flow_1,
                                  .width_factor = width_1,
                                  .spiralize = no_spiralize,
                                  .speed_factor = speed_1 },
                       GCodePath{ .config = travel_config,
                                  .mesh = mesh,
                                  .space_fill_type = SpaceFillType::Lines,
                                  .flow = flow_1,
                                  .width_factor = width_1,
                                  .spiralize = no_spiralize,
                                  .speed_factor = speed_1 },
                       GCodePath{ .config = extrusion_config,
                                  .mesh = mesh,
                                  .space_fill_type = SpaceFillType::Lines,
                                  .flow = flow_1,
                                  .width_factor = width_1,
                                  .spiralize = no_spiralize,
                                  .speed_factor = speed_1 },
                       GCodePath{ .config = travel_config,
                                  .mesh = mesh,
                                  .space_fill_type = SpaceFillType::Lines,
                                  .flow = flow_1,
                                  .width_factor = width_1,
                                  .spiralize = no_spiralize,
                                  .speed_factor = speed_1 },
                       GCodePath{ .config = extrusion_config,
                                  .mesh = mesh,
                                  .space_fill_type = SpaceFillType::Lines,
                                  .flow = flow_1,
                                  .width_factor = width_1,
                                  .spiralize = no_spiralize,
                                  .speed_factor = speed_1 } });
        lines[0].points = { Point(0, 0), Point(1000, 0) };
        lines[1].points = { Point(1000, 0), Point(1000, 400) };
        lines[2].points = { Point(1000, 400), Point(0, 400) };
        lines[3].points = { Point(0, 400), Point(0, 800) };
        lines[4].points = { Point(0, 800), Point(1000, 800) };

        constexpr Ratio flow_12 = 1.2_r;
        constexpr Ratio flow_08 = 0.8_r;
        constexpr Ratio flow_04 = 0.4_r;
        decreasing_flow.assign({ GCodePath{ .config = extrusion_config,
                                            .mesh = mesh,
                                            .space_fill_type = SpaceFillType::Lines,
                                            .flow = flow_12,
                                            .width_factor = width_1,
                                            .spiralize = no_spiralize,
                                            .speed_factor = speed_1 },
                                 GCodePath{ .config = travel_config,
                                            .mesh = mesh,
                                            .space_fill_type = SpaceFillType::Lines,
                                            .flow = flow_1,
                                            .width_factor = width_1,
                                            .spiralize = no_spiralize,
                                            .speed_factor = speed_1 },
                                 GCodePath{ .config = extrusion_config,
                                            .mesh = mesh,
                                            .space_fill_type = SpaceFillType::Lines,
                                            .flow = flow_08,
                                            .width_factor = width_1,
                                            .spiralize = no_spiralize,
                                            .speed_factor = speed_1 },
                                 GCodePath{ .config = travel_config,
                                            .mesh = mesh,
                                            .space_fill_type = SpaceFillType::Lines,
                                            .flow = flow_1,
                                            .width_factor = width_1,
                                            .spiralize = no_spiralize,
                                            .speed_factor = speed_1 },
                                 GCodePath{ .config = extrusion_config,
                                            .mesh = mesh,
                                            .space_fill_type = SpaceFillType::Lines,
                                            .flow = flow_04,
                                            .width_factor = width_1,
                                            .spiralize = no_spiralize,
                                            .speed_factor = speed_1 } });
        decreasing_flow[0].points = { Point(0, 0), Point(1000, 0) };
        decreasing_flow[1].points = { Point(1000, 0), Point(1000, 400) };
        decreasing_flow[2].points = { Point(1000, 400), Point(0, 400) };
        decreasing_flow[3].points = { Point(0, 400), Point(0, 800) };
        decreasing_flow[4].points = { Point(0, 800), Point(1000, 800) };

        constexpr Ratio speed_12 = 1.2_r;
        constexpr Ratio speed_08 = 0.8_r;
        constexpr Ratio speed_04 = 0.4_r;
        decreasing_speed.assign({ GCodePath{ .config = extrusion_config,
                                             .mesh = mesh,
                                             .space_fill_type = SpaceFillType::Lines,
                                             .flow = flow_1,
                                             .width_factor = width_1,
                                             .spiralize = no_spiralize,
                                             .speed_factor = speed_12 },
                                  GCodePath{ .config = travel_config,
                                             .mesh = mesh,
                                             .space_fill_type = SpaceFillType::Lines,
                                             .flow = flow_1,
                                             .width_factor = width_1,
                                             .spiralize = no_spiralize,
                                             .speed_factor = speed_1 },
                                  GCodePath{ .config = extrusion_config,
                                             .mesh = mesh,
                                             .space_fill_type = SpaceFillType::Lines,
                                             .flow = flow_1,
                                             .width_factor = width_1,
                                             .spiralize = no_spiralize,
                                             .speed_factor = speed_08 },
                                  GCodePath{ .config = travel_config,
                                             .mesh = mesh,
                                             .space_fill_type = SpaceFillType::Lines,
                                             .flow = flow_1,
                                             .width_factor = width_1,
                                             .spiralize = no_spiralize,
                                             .speed_factor = speed_1 },
                                  GCodePath{ .config = extrusion_config,
                                             .mesh = mesh,
                                             .space_fill_type = SpaceFillType::Lines,
                                             .flow = flow_1,
                                             .width_factor = width_1,
                                             .spiralize = no_spiralize,
                                             .speed_factor = speed_04 } });
        decreasing_speed[0].points = { Point(0, 0), Point(1000, 0) };
        decreasing_speed[1].points = { Point(1000, 0), Point(1000, 400) };
        decreasing_speed[2].points = { Point(1000, 400), Point(0, 400) };
        decreasing_speed[3].points = { Point(0, 400), Point(0, 800) };
        decreasing_speed[4].points = { Point(0, 800), Point(1000, 800) };

        variable_width.assign({
            GCodePath{ .config = extrusion_config,
                       .mesh = mesh,
                       .space_fill_type = SpaceFillType::Lines,
                       .flow = flow_1,
                       .width_factor = width_1,
                       .spiralize = no_spiralize,
                       .speed_factor = speed_1 },
            GCodePath{ .config = extrusion_config,
                       .mesh = mesh,
                       .space_fill_type = SpaceFillType::Lines,
                       .flow = 0.8_r,
                       .width_factor = width_1,
                       .spiralize = no_spiralize,
                       .speed_factor = speed_1 },
            GCodePath{ .config = extrusion_config,
                       .mesh = mesh,
                       .space_fill_type = SpaceFillType::Lines,
                       .flow = 0.6_r,
                       .width_factor = width_1,
                       .spiralize = no_spiralize,
                       .speed_factor = speed_1 },
            GCodePath{ .config = extrusion_config,
                       .mesh = mesh,
                       .space_fill_type = SpaceFillType::Lines,
                       .flow = 0.4_r,
                       .width_factor = width_1,
                       .spiralize = no_spiralize,
                       .speed_factor = speed_1 },
            GCodePath{ .config = extrusion_config,
                       .mesh = mesh,
                       .space_fill_type = SpaceFillType::Lines,
                       .flow = 0.2_r,
                       .width_factor = width_1,
                       .spiralize = no_spiralize,
                       .speed_factor = speed_1 },
            GCodePath{ .config = extrusion_config,
                       .mesh = mesh,
                       .space_fill_type = SpaceFillType::Lines,
                       .flow = 0.0_r,
                       .width_factor = width_1,
                       .spiralize = no_spiralize,
                       .speed_factor = speed_1 },
        });
        variable_width[0].points = { Point(0, 0), Point(1000, 0) };
        variable_width[1].points = { Point(1000, 0), Point(2000, 0) };
        variable_width[2].points = { Point(2000, 0), Point(3000, 0) };
        variable_width[3].points = { Point(3000, 0), Point(4000, 0) };
        variable_width[4].points = { Point(4000, 0), Point(5000, 0) };
        variable_width[5].points = { Point(5000, 0), Point(6000, 0) };
    }
};
// NOLINTEND(misc-non-private-member-variables-in-classes)

static ExtruderPlanTestPathCollection path_collection;

/*!
 * Tests in this class get parameterized with a vector of GCodePaths to put in
 * the extruder plan, and an extruder plan to put it in.
 */
class ExtruderPlanPathsParameterizedTest : public testing::TestWithParam<std::vector<GCodePath>>
{
public:
    /*!
     * An extruder plan that can be used as a victim for testing.
     */
    ExtruderPlan extruder_plan;

    /*!
     * Flow rate error margins allowed.
     *
     * Floating point arithmetic introduces minute errors which are irrelevant.
     * As long as it's within a millionth, no extruder will be able to render
     * the difference.
     */
    static constexpr double error_margin = 0.000001;

    ExtruderPlanPathsParameterizedTest()
        : extruder_plan(
            /*extruder=*/0,
            /*layer_nr=*/50,
            /*is_initial_layer=*/false,
            /*is_raft_layer=*/false,
            /*layer_thickness=*/100,
            FanSpeedLayerTimeSettings(),
            RetractionConfig())
    {
    }

    /*!
     * Helper method to calculate the flow rate of a path in mm3 per second,
     * minus the influence of flow rate and ignoring any user specified speed
     * alteration other than the back pressure compensation.
     * \param path The path to calculate the flow rate of.
     * \return The flow rate, in cubic millimeters per second.
     */
    [[nodiscard]] static double calculatePathWidth(const GCodePath& path)
    {
        return path.getExtrusionMM3perMM() / path.config.getFlowRatio() / path.flow * path.config.getSpeed() * path.speed_back_pressure_factor;
    }

    [[nodiscard]] static bool shouldCountPath(const GCodePath& path)
    {
        return path.flow > 0.0 && path.width_factor > 0.0 && path.config.getFlowRatio() > 0.0 && path.config.getLineWidth() > 0 && ! path.config.isTravelPath()
            && ! path.config.isBridgePath();
    }
};

INSTANTIATE_TEST_SUITE_P(
    ExtruderPlanTestInstantiation,
    ExtruderPlanPathsParameterizedTest,
    testing::Values(path_collection.square, path_collection.lines, path_collection.decreasing_flow, path_collection.decreasing_speed, path_collection.variable_width));

/*!
 * A fixture for general test cases involving extruder plans.
 *
 * This fixture provides a test extruder plan, without any paths, to test with.
 */
class ExtruderPlanTest : public testing::Test
{
public:
    /*!
     * An extruder plan that can be used as a victim for testing.
     */
    ExtruderPlan extruder_plan;

    ExtruderPlanTest()
        : extruder_plan(
            /*extruder=*/0,
            /*layer_nr=*/50,
            /*is_initial_layer=*/false,
            /*is_raft_layer=*/false,
            /*layer_thickness=*/100,
            FanSpeedLayerTimeSettings(),
            RetractionConfig())
    {
    }
};

/*!
 * Tests that paths remain unmodified if applying back pressure compensation
 * with factor 0.
 */
TEST_P(ExtruderPlanPathsParameterizedTest, BackPressureCompensationZeroIsUncompensated)
{
    extruder_plan.paths = GetParam();
    std::vector<Ratio> original_widths;
    std::vector<Ratio> original_speeds;
    for (const GCodePath& path : extruder_plan.paths)
    {
        original_widths.push_back(path.width_factor);
        original_speeds.push_back(path.speed_factor);
    }

    extruder_plan.applyBackPressureCompensation(0.0_r);

    ASSERT_EQ(extruder_plan.paths.size(), original_widths.size()) << "Number of paths may not have changed.";
    for (size_t i = 0; i < extruder_plan.paths.size(); ++i)
    {
        EXPECT_NEAR(original_widths[i], extruder_plan.paths[i].width_factor, error_margin) << "The width did not change. Back pressure compensation doesn't adjust line width.";
        EXPECT_NEAR(original_speeds[i], extruder_plan.paths[i].speed_factor, error_margin) << "The speed factor did not change, since the compensation factor was 0.";
    }
}

/*!
 * Tests that a factor of 1 causes the back pressure compensation to be
 * completely equalizing the flow rate.
 */
TEST_P(ExtruderPlanPathsParameterizedTest, BackPressureCompensationFull)
{
    extruder_plan.paths = GetParam();
    extruder_plan.applyBackPressureCompensation(1.0_r);

    auto first_extrusion = std::find_if(
        extruder_plan.paths.begin(),
        extruder_plan.paths.end(),
        [&](GCodePath& path)
        {
            return shouldCountPath(path);
        });
    if (first_extrusion == extruder_plan.paths.end()) // Only travel moves in this plan.
    {
        return;
    }
    // All flow rates must be equal to this one.
    const double first_flow_mm3_per_sec = calculatePathWidth(*first_extrusion);

    for (GCodePath& path : extruder_plan.paths)
    {
        if (! shouldCountPath(path))
        {
            continue; // Ignore travel moves.
        }
        const double flow_mm3_per_sec = calculatePathWidth(path);
        EXPECT_NEAR(flow_mm3_per_sec, first_flow_mm3_per_sec, error_margin)
            << "Every path must have a flow rate equal to the first, since the flow changes were completely compensated for.";
    }
}

/*!
 * Tests that a factor of 0.5 halves the differences in flow rate.
 */
TEST_P(ExtruderPlanPathsParameterizedTest, BackPressureCompensationHalf)
{
    extruder_plan.paths = GetParam();

    // Calculate what the flow rates were originally.
    std::vector<double> original_flows;
    for (GCodePath& path : extruder_plan.paths)
    {
        if (! shouldCountPath(path))
        {
            continue; // Ignore travel moves.
        }
        original_flows.push_back(calculatePathWidth(path));
    }
    const auto original_average = std::accumulate(original_flows.begin(), original_flows.end(), 0.0L) / original_flows.size();

    // Apply the back pressure compensation with 50% factor!
    extruder_plan.applyBackPressureCompensation(0.5_r);

    // Calculate the new flow rates.
    std::vector<double> new_flows;
    for (GCodePath& path : extruder_plan.paths)
    {
        if (! shouldCountPath(path))
        {
            continue; // Ignore travel moves.
        }
        new_flows.push_back(calculatePathWidth(path));
    }
    const auto new_average = std::accumulate(new_flows.begin(), new_flows.end(), 0.0L) / new_flows.size();
    // Note that the new average doesn't necessarily need to be the same average! It is most likely a higher average in real-world
    // scenarios.

    // Test that the deviation from the average was halved.
    ASSERT_EQ(original_flows.size(), new_flows.size()) << "We need to have the same number of extrusion moves.";
    for (size_t i = 0; i < new_flows.size(); ++i)
    {
        EXPECT_NEAR((original_flows[i] - original_average) / 2.0, new_flows[i] - new_average, error_margin)
            << "The differences in flow rate needs to be approximately halved, within margin of rounding errors.";
    }
}

/*!
 * Tests back pressure compensation on an extruder plan that is completely
 * empty.
 */
TEST_F(ExtruderPlanTest, BackPressureCompensationEmptyPlan)
{
    // The extruder plan starts off empty. So immediately try applying back-pressure compensation.
    extruder_plan.applyBackPressureCompensation(0.5_r);

    EXPECT_TRUE(extruder_plan.paths.empty()) << "The paths in the extruder plan should remain empty. Also it shouldn't crash.";
}
} // namespace cura
// NOLINTEND(*-magic-numbers)
