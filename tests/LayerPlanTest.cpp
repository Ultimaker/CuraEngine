//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include "../src/Application.h" //To provide settings for the layer plan.
#include "../src/pathPlanning/Comb.h" //To create a combing path around the layer plan.
#include "../src/LayerPlan.h" //The code under test.
#include "../src/RetractionConfig.h" //To provide retraction settings.
#include "../src/Slice.h" //To provide settings for the layer plan.
#include "../src/sliceDataStorage.h" //To provide slice data as input for the planning stage.

namespace cura
{

/*!
 * A fixture to test layer plans with.
 *
 * This fixture gets the previous location initialised to 0,0. You can
 * optionally fill it with some layer data.
 */
class LayerPlanTest : public testing::Test
{
public:
    /*!
     * Cooling settings, which are passed to the layer plan by reference.
     *
     * One for each extruder. There is only one extruder by default in this
     * fixture.
     *
     * \note This needs to be allocated BEFORE layer_plan since the constructor
     * of layer_plan in the initializer list needs to have a valid vector
     * reference.
     */
    std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings;

    /*!
     * Sliced layers divided up into regions for each structure.
     */
    SliceDataStorage* storage;

    /*!
     * A pre-filled layer plan.
     */
    LayerPlan layer_plan;

    /*!
     * A shortcut to easily modify settings in a test.
     */
    Settings* settings;

    /*!
     * An empty mesh. You need to add the layers you want to test with yourself.
     *
     * Necessary to get per-mesh settings. This fixture doesn't add per-mesh
     * setting overrides, so the settings are just the global ones.
     */
    Mesh mesh;

    LayerPlanTest() :
        storage(setUpStorage()),
        layer_plan(
            *storage,
            /*layer_nr=*/100,
            /*z=*/10000,
            /*layer_thickness=*/100,
            /*extruder_nr=*/0,
            fan_speed_layer_time_settings,
            /*comb_boundary_offset=*/20,
            /*comb_move_inside_distance=*/10,
            /*travel_avoid_distance=*/5000
        )
    {
    }

    /*!
     * Prepares the slice data storage before passing it to the layer plan.
     *
     * In order to prepare the slice data storage, the Application class is also
     * initialized with a proper current slice and all of the settings it needs.
     *
     * This needs to be done in a separate function so that it can be executed
     * in the initializer list.
     * \return That same SliceDataStorage.
     */
    SliceDataStorage* setUpStorage()
    {
        constexpr size_t num_mesh_groups = 1;
        Application::getInstance().current_slice = new Slice(num_mesh_groups);

        //Define all settings in the mesh group. The extruder train and model settings will fall back on that then.
        settings = &Application::getInstance().current_slice->scene.current_mesh_group->settings;
        //Default settings. These are not (always) the FDM printer defaults, but sometimes just setting values that can be recognised uniquely as much as possible.
        settings->add("acceleration_prime_tower", "5008");
        settings->add("acceleration_skirt_brim", "5007");
        settings->add("acceleration_support_bottom", "5005");
        settings->add("acceleration_support_infill", "5009");
        settings->add("acceleration_support_roof", "5004");
        settings->add("acceleration_travel", "5006");
        settings->add("adhesion_extruder_nr", "0");
        settings->add("adhesion_type", "brim");
        settings->add("cool_fan_full_layer", "3");
        settings->add("cool_fan_speed_0", "0");
        settings->add("cool_fan_speed_min", "75");
        settings->add("cool_fan_speed_max", "100");
        settings->add("cool_min_speed", "10");
        settings->add("cool_min_layer_time", "5");
        settings->add("cool_min_layer_time_fan_speed_max", "10");
        settings->add("initial_layer_line_width_factor", "1.0");
        settings->add("jerk_prime_tower", "5.8");
        settings->add("jerk_skirt_brim", "5.7");
        settings->add("jerk_support_bottom", "5.5");
        settings->add("jerk_support_infill", "5.9");
        settings->add("jerk_support_roof", "5.4");
        settings->add("jerk_travel", "5.6");
        settings->add("layer_height", "0.1");
        settings->add("layer_start_x", "0");
        settings->add("layer_start_y", "0");
        settings->add("limit_support_retractions", "false");
        settings->add("machine_center_is_zero", "false");
        settings->add("machine_depth", "1000");
        settings->add("machine_extruders_share_nozzle", "false");
        settings->add("machine_height", "1000");
        settings->add("machine_nozzle_tip_outer_diameter", "1");
        settings->add("machine_width", "1000");
        settings->add("material_flow_layer_0", "100");
        settings->add("meshfix_maximum_travel_resolution", "0");
        settings->add("prime_tower_enable", "true");
        settings->add("prime_tower_flow", "108");
        settings->add("prime_tower_line_width", "0.48");
        settings->add("prime_tower_min_volume", "10");
        settings->add("prime_tower_size", "40");
        settings->add("raft_base_line_width", "0.401");
        settings->add("raft_base_acceleration", "5001");
        settings->add("raft_base_jerk", "5.1");
        settings->add("raft_base_speed", "51");
        settings->add("raft_base_thickness", "0.101");
        settings->add("raft_interface_acceleration", "5002");
        settings->add("raft_interface_jerk", "5.2");
        settings->add("raft_interface_line_width", "0.402");
        settings->add("raft_interface_speed", "52");
        settings->add("raft_interface_thickness", "0.102");
        settings->add("raft_surface_acceleration", "5003");
        settings->add("raft_surface_jerk", "5.3");
        settings->add("raft_surface_line_width", "0.403");
        settings->add("raft_surface_speed", "53");
        settings->add("raft_surface_thickness", "0.103");
        settings->add("retraction_amount", "8");
        settings->add("retraction_combing", "off");
        settings->add("retraction_count_max", "30");
        settings->add("retraction_enable", "false");
        settings->add("retraction_extra_prime_amount", "1");
        settings->add("retraction_extrusion_window", "10");
        settings->add("retraction_hop", "1.5");
        settings->add("retraction_hop_enabled", "false");
        settings->add("retraction_hop_only_when_collides", "false");
        settings->add("retraction_min_travel", "0");
        settings->add("retraction_prime_speed", "12");
        settings->add("retraction_retract_speed", "11");
        settings->add("skirt_brim_line_width", "0.47");
        settings->add("skirt_brim_material_flow", "107");
        settings->add("skirt_brim_speed", "57");
        settings->add("speed_prime_tower", "58");
        settings->add("speed_slowdown_layers", "1");
        settings->add("speed_support_bottom", "55");
        settings->add("speed_support_infill", "59");
        settings->add("speed_support_roof", "54");
        settings->add("speed_travel", "56");
        settings->add("support_bottom_extruder_nr", "0");
        settings->add("support_bottom_line_width", "0.405");
        settings->add("support_bottom_material_flow", "105");
        settings->add("support_infill_extruder_nr", "0");
        settings->add("support_line_width", "0.49");
        settings->add("support_material_flow", "109");
        settings->add("support_roof_extruder_nr", "0");
        settings->add("support_roof_line_width", "0.404");
        settings->add("support_roof_material_flow", "104");
        settings->add("wall_line_count", "3");
        settings->add("wall_line_width_x", "0.3");
        settings->add("wall_line_width_0", "0.301");

        Application::getInstance().current_slice->scene.extruders.emplace_back(0, settings); //Add an extruder train.

        //Set the fan speed layer time settings (since the LayerPlan constructor copies these).
        FanSpeedLayerTimeSettings fan_settings;
        fan_settings.cool_min_layer_time = settings->get<Duration>("cool_min_layer_time");
        fan_settings.cool_min_layer_time_fan_speed_max = settings->get<Duration>("cool_min_layer_time_fan_speed_max");
        fan_settings.cool_fan_speed_0 = settings->get<Ratio>("cool_fan_speed_0");
        fan_settings.cool_fan_speed_min = settings->get<Ratio>("cool_fan_speed_min");
        fan_settings.cool_fan_speed_max = settings->get<Ratio>("cool_fan_speed_max");
        fan_settings.cool_min_speed = settings->get<Velocity>("cool_min_speed");
        fan_settings.cool_fan_full_layer = settings->get<LayerIndex>("cool_fan_full_layer");
        fan_speed_layer_time_settings.push_back(fan_settings);

        //Set the retraction settings (also copied by LayerPlan).
        RetractionConfig retraction_config;
        retraction_config.distance = settings->get<double>("retraction_amount");
        retraction_config.prime_volume = settings->get<double>("retraction_extra_prime_amount");
        retraction_config.speed = settings->get<Velocity>("retraction_retract_speed");
        retraction_config.primeSpeed = settings->get<Velocity>("retraction_prime_speed");
        retraction_config.zHop = settings->get<coord_t>("retraction_hop");
        retraction_config.retraction_min_travel_distance = settings->get<coord_t>("retraction_min_travel");
        retraction_config.retraction_extrusion_window = settings->get<double>("retraction_extrusion_window");
        retraction_config.retraction_count_max = settings->get<size_t>("retraction_count_max");

        SliceDataStorage* result = new SliceDataStorage();
        result->retraction_config_per_extruder[0] = retraction_config;
        return result;
    }

    void SetUp()
    {
        layer_plan.addTravel_simple(Point(0, 0)); //Make sure that it appears as if we have already done things in this layer plan. Just the standard case.
    }

    /*!
     * Cleaning up after a test is hardly necessary but just for neatness.
     */
    void TearDown()
    {
        delete storage;
        delete Application::getInstance().current_slice;
    }
};

//Test all combinations of these settings in parameterised tests.
std::vector<std::string> retraction_enable = {"false", "true"};
std::vector<std::string> hop_enable = {"false", "true"};
std::vector<std::string> combing = {"off", "all"};
std::vector<bool> is_long = {false, true}; //Whether or not the travel move is longer than retraction_min_travel.
std::vector<bool> is_long_combing = {false, true}; //Whether or not the total travel distance is longer than retraction_combing_max_distance.
enum AddTravelTestScene {
    OPEN, //The travel move goes through open air. There's nothing in the entire layer.
    INSIDE, //The travel move goes through a part on the inside.
    OBSTRUCTION, //The travel move goes through open air, but there's something in the way that needs to be avoided.
    INSIDE_OBSTRUCTION, //The travel move goes through the inside of a part, but there's a hole in the way that needs to be avoided.
    OTHER_PART //The travel move goes from one part to another.
};
std::vector<AddTravelTestScene> scene = {OPEN, INSIDE, OBSTRUCTION, INSIDE_OBSTRUCTION, OTHER_PART};

/*!
 * Parameters for the AddTravelTest class.
 *
 * Instead of using a tuple for this, an explicit struct makes the code easier
 * to read since you can get individual parameters by name rather than using
 * std::get<4> or something.
 */
struct AddTravelParameters
{
    std::string retraction_enable;
    std::string hop_enable;
    std::string combing;
    bool is_long;
    bool is_long_combing;
    AddTravelTestScene scene;

    /*!
     * Unrolls the parameters.
     * \param parameters The parameters as received by testing::Combine().
     */
    AddTravelParameters(const std::tuple<std::string, std::string, std::string, bool, bool, AddTravelTestScene>& parameters)
    {
        retraction_enable = std::get<0>(parameters);
        hop_enable = std::get<1>(parameters);
        combing = std::get<2>(parameters);
        is_long = std::get<3>(parameters);
        is_long_combing = std::get<4>(parameters);
        scene = std::get<5>(parameters);
    }

    /*!
     * Return whether the travel move goes through open air (outside of parts).
     * \return Whether the travel move goes through open air (outside of parts).
     */
    bool throughOutside() const
    {
        return scene == AddTravelTestScene::OPEN || scene == AddTravelTestScene::OBSTRUCTION || scene == AddTravelTestScene::OTHER_PART;
    }

    /*!
     * Return whether the travel move is forced to penetrate any walls.
     * \return Whether the travel move is forced to penetrate any walls.
     */
    bool throughWalls() const
    {
        return scene == AddTravelTestScene::OTHER_PART;
    }
};

/*!
 * Parameterised testing class that combines many combinations of cases to test
 * travel moves in. The parameters are in the same order as above:
 * 1. retraction_enable
 * 2. hop_enable
 * 3. combing mode
 * 4. Long travel move.
 * 5. Long travel move (combing).
 * 6. Scene.
 */
class AddTravelTest : public LayerPlanTest, public testing::WithParamInterface<std::tuple<std::string, std::string, std::string, bool, bool, AddTravelTestScene>>
{
public:
    //Parameters to test with.
    AddTravelParameters parameters;

    //Some obstacles that can be placed in the scene.
    Polygon around_start_end; //Around both the start and end position.
    Polygon around_start; //Only around the start position.
    Polygon around_end; //Only around the end position.
    Polygon between; //Between the start and end position.
    Polygon between_hole; //Negative polygon between the start and end position (a hole).

    AddTravelTest() : parameters(std::make_tuple<std::string, std::string, std::string, bool, bool, AddTravelTestScene>("false", "false", "off", false, false, AddTravelTestScene::OPEN))
    {
        around_start_end.add(Point(-100, -100));
        around_start_end.add(Point(500100, -100));
        around_start_end.add(Point(500100, 500100));
        around_start_end.add(Point(-100, 500100));

        around_start.add(Point(-100, -100));
        around_start.add(Point(100, -100));
        around_start.add(Point(100, 100));
        around_start.add(Point(-100, 100));

        around_end.add(Point(249900, 249900));
        around_end.add(Point(250100, 249900));
        around_end.add(Point(250100, 250100));
        around_end.add(Point(249900, 249900));

        between.add(Point(250000, 240000));
        between.add(Point(260000, 240000));
        between.add(Point(260000, 300000));
        between.add(Point(250000, 300000));

        between_hole.add(Point(250000, 240000));
        between_hole.add(Point(250000, 300000));
        between_hole.add(Point(260000, 300000));
        between_hole.add(Point(260000, 240000));
    }

    /*!
     * Runs the actual test, adding a travel move to the layer plan with the
     * specified parameters.
     * \param parameters The parameter object provided to the test.
     * \return The resulting g-code path.
     */
    GCodePath run(const std::tuple<std::string, std::string, std::string, bool, bool, AddTravelTestScene>& combine_parameters)
    {
        parameters = AddTravelParameters(combine_parameters);
        settings->add("retraction_enable", parameters.retraction_enable);
        settings->add("retraction_hop_enabled", parameters.hop_enable);
        settings->add("retraction_combing", parameters.combing);
        settings->add("retraction_min_travel", parameters.is_long ? "1" : "10000"); //If disabled, give it a high minimum travel so we're sure that our travel move is shorter.
        storage->retraction_config_per_extruder[0].retraction_min_travel_distance = settings->get<coord_t>("retraction_min_travel"); //Update the copy that the storage has of this.
        settings->add("retraction_combing_max_distance", parameters.is_long_combing ? "1" : "10000");

        Polygons slice_data;
        switch(parameters.scene)
        {
            case OPEN:
                layer_plan.setIsInside(false);
                layer_plan.was_inside = false;
                break;
            case INSIDE:
                slice_data.add(around_start_end);
                layer_plan.setIsInside(true);
                layer_plan.was_inside = true;
                break;
            case OBSTRUCTION:
                slice_data.add(between);
                layer_plan.setIsInside(false);
                layer_plan.was_inside = false;
                break;
            case INSIDE_OBSTRUCTION:
                slice_data.add(around_start_end);
                slice_data.add(between_hole);
                layer_plan.setIsInside(true);
                layer_plan.was_inside = true;
                break;
            case OTHER_PART:
                slice_data.add(around_start);
                slice_data.add(around_end);
                layer_plan.setIsInside(true);
                layer_plan.was_inside = true;
                break;
        }
        layer_plan.comb_boundary_minimum = slice_data;
        layer_plan.comb_boundary_preferred = slice_data; //We don't care about the combing accuracy itself, so just use the same for both.
        if(parameters.combing != "off")
        {
            layer_plan.comb = new Comb(*storage, /*layer_nr=*/100, layer_plan.comb_boundary_minimum, layer_plan.comb_boundary_preferred, /*comb_boundary_offset=*/20, /*travel_avoid_distance=*/5000, /*comb_move_inside_distance=*/10);
        }
        else
        {
            layer_plan.comb = nullptr;
        }

        const Point destination(500000, 500000);
        return layer_plan.addTravel(destination);
    }
};

INSTANTIATE_TEST_CASE_P(AllCombinations, AddTravelTest, testing::Combine(
    testing::ValuesIn(retraction_enable),
    testing::ValuesIn(hop_enable),
    testing::ValuesIn(combing),
    testing::ValuesIn(is_long),
    testing::ValuesIn(is_long_combing),
    testing::ValuesIn(scene)
));

/*!
 * Test if there are indeed no retractions if retractions are disabled.
 */
TEST_P(AddTravelTest, NoRetractionIfDisabled)
{
    const GCodePath result = run(GetParam());

    if(parameters.retraction_enable == "false")
    {
        EXPECT_FALSE(result.retract) << "If retraction is disabled it should not retract.";
    }
}

/*!
 * Test if there are indeed no Z hops if they are disabled.
 */
TEST_P(AddTravelTest, NoHopIfDisabled)
{
    const GCodePath result = run(GetParam());

    if(parameters.hop_enable == "false")
    {
        EXPECT_FALSE(result.perform_z_hop) << "If Z hop is disabled it should not hop.";
    }
}

/*!
 * Test if there are no retractions if the travel move is short, regardless of
 * whether retractions are enabled or not.
 */
TEST_P(AddTravelTest, NoRetractionIfShort)
{
    const GCodePath result = run(GetParam());

    if(!parameters.is_long)
    {
        EXPECT_FALSE(result.retract) << "If the travel move is shorter than retraction_min_travel, it should not retract.";
    }
}

/*!
 * Tests that without combing we do a retraction (if enabled).
 */
TEST_P(AddTravelTest, NoCombingRetraction)
{
    const GCodePath result = run(GetParam());

    if(parameters.retraction_enable == "true" && parameters.combing == "off" && parameters.is_long)
    {
        EXPECT_TRUE(result.retract) << "If we don't do combing, we should always retract since we aren't even checking if any walls are crossed.";
    }
}

/*!
 * Tests that we don't retract for short combing moves.
 */
TEST_P(AddTravelTest, NoRetractionShortCombing)
{
    const GCodePath result = run(GetParam());

    if(parameters.combing != "off" && !parameters.is_long_combing && !parameters.throughOutside())
    {
        EXPECT_FALSE(result.retract) << "Combing move is shorter than the retraction_combing_max_distance, so it shouldn't retract.";
    }
}

/*!
 * Tests that we do retract for long combing moves (if enabled).
 */
TEST_P(AddTravelTest, RetractionLongCombing)
{
    const GCodePath result = run(GetParam());

    //  Combing is enabled              combing move is longer  not too short to not retract         retraction enabled
    if(parameters.combing != "off" && parameters.is_long_combing && parameters.is_long && parameters.retraction_enable == "true")
    {
        EXPECT_TRUE(result.retract) << "Combing move is longer than the retraction_combing_max_distance, so it should retract.";
    }
}

/*!
 * Tests that we always hop if retracting (if enabled), except for long combing
 * moves.
 */
TEST_P(AddTravelTest, HopWhenRetracting)
{
    const GCodePath result = run(GetParam());

    if(result.retract)
    {
        if(parameters.combing != "off" && parameters.is_long_combing && !parameters.throughWalls() && !parameters.throughOutside())
        {
            EXPECT_FALSE(result.perform_z_hop) << "If combing without hitting any walls, it should not hop, but if the combing move is long it might still retract.";
        }
        else if(parameters.hop_enable == "true")
        {
            EXPECT_TRUE(result.perform_z_hop) << "If hop is enabled and we retract, we must also hop.";
        }
        else
        {
            EXPECT_FALSE(result.perform_z_hop) << "Hop is disabled so even though we retract we will not hop.";
        }
    }
}

/*!
 * Tests that we make a retraction if combing is enabled but fails to find a
 * valid path.
 */
TEST_P(AddTravelTest, RetractIfCombingImpossible)
{
    const GCodePath result = run(GetParam());

    if(parameters.retraction_enable != "true" || !parameters.is_long || parameters.combing == "off")
    {
        return; //Not interested if retraction is disabled, the move is too short to retract or it's not combing.
    }
    if(parameters.throughOutside())
    {
        EXPECT_TRUE(result.retract) << "If traveling through air, it should retract.";
    }
    else if(!parameters.is_long_combing)
    {
        EXPECT_FALSE(result.retract) << "If combing is possible, it should not retract (unless the travel move is too long).";
    }
}

/*!
 * Tests to verify that when there is no retraction, then there should also be no unretraction before the last travel
 * move in the path.
 */
TEST_P(AddTravelTest, NoUnretractBeforeLastTravelMoveIfNoPriorRetraction)
{
    const GCodePath result = run(GetParam());

    if(!result.retract)
    {
        EXPECT_FALSE(result.unretract_before_last_travel_move) << "If no retraction has been issued, then there should also be no unretraction before the last travel move.";
    }
}

}
