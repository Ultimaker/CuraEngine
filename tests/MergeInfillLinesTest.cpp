//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include "../src/Application.h" //To set up a scene and load settings that the layer plan and merger need.
#include "../src/FanSpeedLayerTime.h" //Required to construct a layer plan. Doesn't influence our tests.
#include "../src/pathPlanning/GCodePath.h" //The paths that we're going to be merging.
#include "../src/LayerPlan.h" //Constructing plans that the mergers can merge lines in.
#include "../src/MergeInfillLines.h" //The class under test.
#include "../src/RetractionConfig.h" //Required to construct a layer plan. Doesn't influence our tests.
#include "../src/Slice.h" //To set up a scene and load settings that the layer plan and merger need.
#include "../src/settings/types/LayerIndex.h" //Required to construct a layer plan. Doesn't influence our tests.

namespace cura
{

class MergeInfillLinesTest : public testing::Test
{
public:
    //These settings don't matter for this test so they are all the same for every fixture.
    const size_t extruder_nr = 0;
    const LayerIndex layer_nr = 0;
    const bool is_initial_layer = false;
    const bool is_raft_layer = false;
    const coord_t layer_thickness = 100;

    /*
     * A merger with a layer plan that contains no paths at all.
     */
    ExtruderPlan* empty_plan;
    MergeInfillLines* empty_plan_merger;

    /*
     * These fields are required for constructing layer plans and must be held
     * constant for as long as the lifetime of the plans. Construct them once
     * and store them in this fixture class.
     */
    const FanSpeedLayerTimeSettings fan_speed_layer_time;
    const RetractionConfig retraction_config;
    const GCodePathConfig skin_config;

    /*
     * A path of skin lines without any points.
     */
    GCodePath empty_skin;

    /*
     * A path of a single skin line.
     */
    GCodePath single_skin;

    MergeInfillLinesTest()
     : fan_speed_layer_time()
     , retraction_config()
     , skin_config(PrintFeatureType::Skin, 400, layer_thickness, 1, GCodePathConfig::SpeedDerivatives{50, 1000, 10})
     , empty_skin(skin_config, "merge_infill_lines_mesh", SpaceFillType::None, 1.0, false)
     , single_skin(skin_config, "merge_infill_lines_mesh", SpaceFillType::Lines, 1.0, false)
    {
         single_skin.points.emplace_back(1000, 0);
    }

    void SetUp()
    {
        //Set up a scene so that we may request settings.
        Application::getInstance().current_slice = new Slice(1);
        Application::getInstance().current_slice->scene.extruders.emplace_back(0, nullptr);
        ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders.back();
        train.settings.add("machine_nozzle_size", "0.4");
        train.settings.add("meshfix_maximum_deviation", "0.1");

        empty_plan = new ExtruderPlan(extruder_nr, layer_nr, is_initial_layer, is_raft_layer, layer_thickness, fan_speed_layer_time, retraction_config);
        empty_plan_merger = new MergeInfillLines(*empty_plan);
    }

    void TearDown()
    {
        delete empty_plan;
        delete empty_plan_merger;
        delete Application::getInstance().current_slice;
    }
};

TEST_F(MergeInfillLinesTest, CalcPathLengthEmpty)
{
    const coord_t result = empty_plan_merger->calcPathLength(Point(0, 0), empty_skin);
    EXPECT_EQ(result, 0);
}

TEST_F(MergeInfillLinesTest, CalcPathLengthSingle)
{
    const coord_t result = empty_plan_merger->calcPathLength(Point(0, 0), single_skin);
    EXPECT_EQ(result, 1000);
}

} //namespace cura