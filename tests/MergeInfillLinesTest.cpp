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
    const GCodePathConfig travel_config;

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

    /*
     * Basic zigzag of skin lines.
     * There are 11 lines with travel moves in between them. The lines are 100
     * microns long and 400 microns wide. They should get merged to one long
     * line of 100 microns wide and 4400 microns long.
     */
    std::vector<GCodePath> zigzag;

    MergeInfillLinesTest()
     : starting_position(0, 0)
     , fan_speed_layer_time()
     , retraction_config()
     , skin_config(PrintFeatureType::Skin, 400, layer_thickness, 1, GCodePathConfig::SpeedDerivatives{50, 1000, 10})
     , travel_config(PrintFeatureType::MoveCombing, 0, layer_thickness, 0, GCodePathConfig::SpeedDerivatives{100, 1000, 10})
     , empty_skin(skin_config, "merge_infill_lines_mesh", SpaceFillType::None, 1.0, false)
     , single_skin(skin_config, "merge_infill_lines_mesh", SpaceFillType::Lines, 1.0, false)
     , lengthwise_skin(skin_config, "merge_infill_lines_mesh", SpaceFillType::Lines, 1.0, false)
    {
        single_skin.points.emplace_back(1000, 0);

        lengthwise_skin.points = {Point(1000, 0),
                                  Point(2000, 0),
                                  Point(3000, 0),
                                  Point(4000, 0)};

        //Create the zigzag.
        constexpr Ratio normal_flow = 1.0;
        constexpr bool no_spiralize = false;
        constexpr size_t num_lines = 10;
        //Creates a zig-zag line with extrusion moves when moving in the Y direction and travel moves in between:
        //  _   _   _
        // | |_| |_| |_|
        for(size_t i = 0; i < num_lines; i++)
        {
            zigzag.emplace_back(skin_config, "merge_infill_lines_mesh", SpaceFillType::Lines, normal_flow, no_spiralize);
            zigzag.back().points.emplace_back(400 * i, 100 * ((i + 1) % 2));
            zigzag.emplace_back(travel_config, "merge_infill_lines_mesh", SpaceFillType::None, normal_flow, no_spiralize);
            zigzag.back().points.emplace_back(400 * (i + 1), 100 * ((i + 1) % 2));
        }
        //End with an extrusion move, not a travel move.
        zigzag.emplace_back(skin_config, "merge_infill_lines_mesh", SpaceFillType::Lines, normal_flow, no_spiralize);
        zigzag.back().points.emplace_back(400 * num_lines, 100 * ((num_lines + 1) % 2));
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

/*
 * Tries merging a bunch of parallel lines with travel moves in between.
 *
 * This is the basic use case for merging infill lines.
 */
TEST_F(MergeInfillLinesTest, MergeParallel)
{
    const bool result = merger->mergeInfillLines(zigzag, starting_position);

    EXPECT_TRUE(result) << "The simple zig-zag pattern should get merged fine.";
    EXPECT_LE(zigzag.size(), 5); //Some lenience. Ideally it'd be one.
}

/*
 * Tests if the total extruded volume is the same as the original lines.
 */
TEST_F(MergeInfillLinesTest, DISABLED_ExtrudedVolume)
{
    coord_t original_volume = 0;
    Point position = starting_position;
    for(const GCodePath& path : zigzag)
    {
        for(const Point& point : path.points)
        {
            const coord_t length = vSize(point - position);
            original_volume += length * (path.getExtrusionMM3perMM() * 1000000);
            position = point;
        }
    }

    merger->mergeInfillLines(zigzag, starting_position);
    /* If it fails to merge, other tests fail. This test depends on that, but we
    don't necessarily want it to fail as a false negative if merging in general
    fails, because we don't want to think that the volume is wrong then. So we
    don't check the outcome of the merging itself, just the volume. */

    coord_t new_volume = 0;
    position = starting_position;
    for(const GCodePath& path : zigzag)
    {
        for(const Point& point : path.points)
        {
            const coord_t length = vSize(point - position);
            new_volume += length * (path.getExtrusionMM3perMM() * 1000000);
            position = point;
        }
    }

    EXPECT_EQ(original_volume, new_volume);
}

/*
 * Parameterised test for merging infill lines inside thin walls.
 *
 * The thin walls are filled with lines of various rotations. This is the float
 * parameter.
 */
class MergeInfillLinesThinWallsTest : public MergeInfillLinesTest, public testing::WithParamInterface<double>
{
public:
    const coord_t wall_thickness = 200; //0.2mm wall.
};

TEST_P(MergeInfillLinesThinWallsTest, DISABLED_MergeThinWalls)
{
    const AngleRadians rotation = AngleRadians(GetParam()); //Converts degrees to radians!
    constexpr Ratio normal_flow = 1.0;
    constexpr bool no_spiralize = false;
    constexpr size_t num_lines = 10;

    //Construct parallel lines under a certain rotation. It looks like this: /////
    //The perpendicular distance between the lines will be exactly one line width.
    //The distance between the adjacent endpoints of the lines will be greater for steeper angles.
    std::vector<GCodePath> paths;
    const coord_t line_shift = wall_thickness / std::cos(rotation); //How far the top of the line is shifted from the bottom.
    const coord_t line_horizontal_spacing = skin_config.getLineWidth() / std::sin(rotation); //Horizontal spacing between starting vertices.
    for(size_t i = 0; i < num_lines; i++)
    {
        paths.emplace_back(skin_config, "merge_infill_lines_mesh", SpaceFillType::Lines, normal_flow, no_spiralize);
        paths.back().points.emplace_back(line_horizontal_spacing * i + line_shift * ((i + 1) % 2), wall_thickness * ((i + 1) % 2));
        paths.emplace_back(travel_config, "merge_infill_lines_mesh", SpaceFillType::None, normal_flow, no_spiralize);
        paths.back().points.emplace_back(line_horizontal_spacing * (i + 1) + line_shift * ((i + 1) % 2), wall_thickness * ((i + 1) % 2));
    }
    paths.emplace_back(skin_config, "merge_infill_lines_mesh", SpaceFillType::Lines, normal_flow, no_spiralize);
    paths.back().points.emplace_back(line_horizontal_spacing * num_lines + line_shift * ((num_lines + 1) % 2), wall_thickness * ((num_lines + 1) % 2));

    const bool merged = merger->mergeInfillLines(paths, starting_position);

    EXPECT_TRUE(merged) << "These are the test cases where line segments should get merged.";
    EXPECT_LE(paths.size(), 5) << "Should get merged to 1 line, but give a bit of leeway.";
}

INSTANTIATE_TEST_CASE_P(MergeThinWallsTest, MergeInfillLinesThinWallsTest, testing::Range(-45.0, 45.0, 5.0));

} //namespace cura