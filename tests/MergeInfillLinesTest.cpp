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
    const Point starting_position; //All plans start at 0,0.

    /*
     * A merger to test with.
     */
    ExtruderPlan* extruder_plan;
    MergeInfillLines* merger;

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

    /*
     * A path of multiple skin lines that together form a straight line.
     *
     * This path should not get merged together to a single line.
     */
    GCodePath lengthwise_skin;

    MergeInfillLinesTest()
     : starting_position(0, 0)
     , fan_speed_layer_time()
     , retraction_config()
     , skin_config(PrintFeatureType::Skin, 400, layer_thickness, 1, GCodePathConfig::SpeedDerivatives{50, 1000, 10})
     , empty_skin(skin_config, "merge_infill_lines_mesh", SpaceFillType::None, 1.0, false)
     , single_skin(skin_config, "merge_infill_lines_mesh", SpaceFillType::Lines, 1.0, false)
     , lengthwise_skin(skin_config, "merge_infill_lines_mesh", SpaceFillType::Lines, 1.0, false)
    {
         single_skin.points.emplace_back(1000, 0);

         lengthwise_skin.points = {Point(1000, 0),
                                   Point(2000, 0),
                                   Point(3000, 0),
                                   Point(4000, 0)};
    }

    void SetUp()
    {
        //Set up a scene so that we may request settings.
        Application::getInstance().current_slice = new Slice(1);
        Application::getInstance().current_slice->scene.extruders.emplace_back(0, nullptr);
        ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders.back();
        train.settings.add("machine_nozzle_size", "0.4");
        train.settings.add("meshfix_maximum_deviation", "0.1");

        extruder_plan = new ExtruderPlan(extruder_nr, layer_nr, is_initial_layer, is_raft_layer, layer_thickness, fan_speed_layer_time, retraction_config);
        merger = new MergeInfillLines(*extruder_plan);
    }

    void TearDown()
    {
        delete extruder_plan;
        delete merger;
        delete Application::getInstance().current_slice;
    }
};

TEST_F(MergeInfillLinesTest, CalcPathLengthEmpty)
{
    EXPECT_EQ(0, merger->calcPathLength(starting_position, empty_skin));
}

TEST_F(MergeInfillLinesTest, CalcPathLengthSingle)
{
    EXPECT_EQ(1000, merger->calcPathLength(starting_position, single_skin));
}

TEST_F(MergeInfillLinesTest, CalcPathLengthMultiple)
{
    EXPECT_EQ(4000, merger->calcPathLength(starting_position, lengthwise_skin));
}

/*
 * Tries merging an empty set of paths together.
 *
 * This changes nothing in the paths, since there is nothing to change.
 */
TEST_F(MergeInfillLinesTest, MergeEmpty)
{
    std::vector<GCodePath> paths; //Empty. No paths to merge.

    const bool result = merger->mergeInfillLines(paths, starting_position);

    EXPECT_FALSE(result) << "There are no lines to merge.";
    EXPECT_EQ(paths.size(), 0) << "The number of paths should still be zero.";
}

/*
 * Tries merging a single path of a single line.
 *
 * This changes nothing in the paths, since the line cannot be merged with
 * anything else.
 */
TEST_F(MergeInfillLinesTest, MergeSingle)
{
    std::vector<GCodePath> paths;
    paths.push_back(single_skin);

    const bool result = merger->mergeInfillLines(paths, starting_position);

    EXPECT_FALSE(result) << "There is only one line, so it can't be merged with other lines.";
    ASSERT_EQ(paths.size(), 1) << "The path should not get removed.";
    EXPECT_EQ(paths[0].points.size(), 1) << "The path should not be modified.";
}

/*
 * Tries merging a single path that consists of multiple vertices in a straight
 * line.
 *
 * This should not change anything in the paths, since the lines are in a single
 * path without travel moves in between. It's just drawing a curve, and that
 * curve should not get modified.
 *
 * This is basically the case that went wrong with the "Weird Fat Infill" bug
 * (CURA-5776).
 */
TEST_F(MergeInfillLinesTest, MergeLenthwise)
{
    std::vector<GCodePath> paths;
    paths.push_back(lengthwise_skin);

    const bool result = merger->mergeInfillLines(paths, starting_position);

    EXPECT_FALSE(result) << "Patterns like Gyroid infill with many (almost) lengthwise lines should not get merged, even if those lines are short.";
    ASSERT_EQ(paths.size(), 1) << "The path should not get removed or split.";
    EXPECT_EQ(paths[0].points.size(), 4) << "The path should not be modified.";
}

} //namespace cura