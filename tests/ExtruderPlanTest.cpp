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
     * An extruder plan that can be used as a victim for testing.
     */
    ExtruderPlan extruder_plan;

    /*!
     * Configuration to print all paths with in the fixture.
     *
     * This config is referred to via pointer, so changing it will immediately
     * change it for all paths.
     */
    GCodePathConfig path_config;

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
        path_config(
            PrintFeatureType::OuterWall,
            /*line_width=*/400,
            100,//extruder_plan.layer_thickness,
            /*flow=*/1.0_r,
            GCodePathConfig::SpeedDerivatives(50, 1000, 10)
        )
    {
        square.push_back(GCodePath(
            path_config,
            /*mesh_id=*/"test_mesh",
            SpaceFillType::PolyLines,
            /*flow=*/1.0_r,
            /*spiralize=*/false,
            /*speed_factor=*/1.0_r
        ));
        square.back().points = {
            Point(0, 0),
            Point(1000, 0),
            Point(1000, 1000),
            Point(0, 1000),
            Point(0, 0)
        };
    }
};

/*!
 * Tests that paths remain unmodified if applying back pressure compensation
 * with factor 0.
 */
TEST_F(ExtruderPlanTest, BackPressureCompensationZeroIsUncompensated)
{
    extruder_plan.paths = square;
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
