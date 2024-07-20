// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "FffGcodeWriter.h" //Unit under test.

#include <unordered_set>
#include <iostream>
#include <fstream>
#include <filesystem>


#include <range/v3/view/join.hpp>
#include <scripta/logger.h>

#include "geometry/OpenPolyline.h"
#include "geometry/Polygon.h" //To create example polygons.
#include "settings/Settings.h" //Settings to generate walls with.
#include "sliceDataStorage.h" //Sl
#include "LayerPlan.h"
#include "Slice.h"
#include "Application.h"
#include <gtest/gtest.h>
#include <gmock/gmock.h>

// NOLINTBEGIN(*-magic-numbers)
namespace cura
{
/*!
 * Fixture that provides a basis for testing wall computation.
 */
class FffGcodeWriterTest : public testing::Test
{
public:
    Settings* settings;
    FffGcodeWriter fff_gcode_writer;

    Shape square_shape;
    // Square that fits wholly inside the above square
    Shape inner_square;

    FffGcodeWriterTest()
        : fff_gcode_writer()
    {
        square_shape.emplace_back();
        square_shape.back().emplace_back(0, 0);
        square_shape.back().emplace_back(MM2INT(100), 0);
        square_shape.back().emplace_back(MM2INT(100), MM2INT(100));
        square_shape.back().emplace_back(0, MM2INT(100));

        inner_square.emplace_back();
        inner_square.back().emplace_back(MM2INT(10), MM2INT(20));
        inner_square.back().emplace_back(MM2INT(60), MM2INT(20));
        inner_square.back().emplace_back(MM2INT(60), MM2INT(60));
        inner_square.back().emplace_back(MM2INT(10), MM2INT(60));

    }

    SliceDataStorage* setUpStorage()
    {
        constexpr size_t num_mesh_groups = 1;
        Application::getInstance().current_slice_ = new Slice(num_mesh_groups);

        // Define all settings in the mesh group. The extruder train and model settings will fall back on that then.
        settings = &Application::getInstance().current_slice_->scene.current_mesh_group->settings;
        // Default settings. These are not (always) the FDM printer defaults, but sometimes just setting values that can be recognised
        // uniquely as much as possible.


        const auto path = std::filesystem::path(__FILE__).parent_path().append("test_default_settings.txt").string();
        std::ifstream file(path);

        std::string line;
        while (std::getline(file, line)) {
            size_t pos = line.find('=');
            std::string key = line.substr(0, pos);
            std::string value = line.substr(pos + 1);
            settings->add(key, value);
        }    

        Application::getInstance().current_slice_->scene.extruders.emplace_back(0, settings); // Add an extruder train.

        // Set the retraction settings (also copied by LayerPlan).
        RetractionConfig retraction_config;
        retraction_config.distance = settings->get<double>("retraction_amount");
        retraction_config.prime_volume = settings->get<double>("retraction_extra_prime_amount");
        retraction_config.speed = settings->get<Velocity>("retraction_retract_speed");
        retraction_config.primeSpeed = settings->get<Velocity>("retraction_prime_speed");
        retraction_config.zHop = settings->get<coord_t>("retraction_hop");
        retraction_config.retraction_min_travel_distance = settings->get<coord_t>("retraction_min_travel");
        retraction_config.retraction_extrusion_window = settings->get<double>("retraction_extrusion_window");
        retraction_config.retraction_count_max = settings->get<size_t>("retraction_count_max");

        auto* result = new SliceDataStorage();
        result->retraction_wipe_config_per_extruder[0].retraction_config = retraction_config;
        return result;
    }
};

TEST_F(FffGcodeWriterTest, SurfaceGetsExtraInfillLinesUnderIt)
{
    // SETUP

    SliceDataStorage* storage = setUpStorage();
    
    // Set the fan speed layer time settings (since the LayerPlan constructor copies these).
    FanSpeedLayerTimeSettings fan_settings;
    fan_settings.cool_min_layer_time = settings->get<Duration>("cool_min_layer_time");
    fan_settings.cool_min_layer_time_fan_speed_max = settings->get<Duration>("cool_min_layer_time_fan_speed_max");
    fan_settings.cool_fan_speed_0 = settings->get<Ratio>("cool_fan_speed_0");
    fan_settings.cool_fan_speed_min = settings->get<Ratio>("cool_fan_speed_min");
    fan_settings.cool_fan_speed_max = settings->get<Ratio>("cool_fan_speed_max");
    fan_settings.cool_min_speed = settings->get<Velocity>("cool_min_speed");
    fan_settings.cool_fan_full_layer = settings->get<LayerIndex>("cool_fan_full_layer");
    
    Mesh mesh(*settings);

    LayerPlan gcode_layer(*storage, 100, 10000, 100, 0, {fan_settings}, 20, 10, 5000 );
    SliceMeshStorage mesh_storage(&mesh, 200);
    size_t extruder_nr = 0;
    MeshPathConfigs mesh_config(mesh_storage, 10, 100, {0.5});
    SliceLayerPart part;

    part.infill_area_per_combine_per_density = {{square_shape}};
    part.infill_area = square_shape;

    mesh_storage.layers[101].parts.emplace_back();
    SliceLayerPart& top_part = mesh_storage.layers[101].parts.back();
    top_part.skin_parts.emplace_back();
    top_part.skin_parts[0].outline.push_back(inner_square);

    // The actual thing we're testing
    //  We have set up a large square of infill on layer 100
    //  And a smaller square of skin on layer 101
    //  We're expecting the sparse infill on layer 100 to have
    //  some lines to support the corners of the layer above.
    //  But we arent wanting the whole area densely supported.
    fff_gcode_writer.processSingleLayerInfill(
        *storage,
        gcode_layer,
        mesh_storage,
        extruder_nr,
        mesh_config,
        part
    );

    // Test helper
    auto checkPointIsPassed = [&](Point2LL p, coord_t margin)-> bool {
        Point2LL last;
        for (const auto& path:gcode_layer.extruder_plans_[0].paths_) {
            for (const auto& point: path.points) {
                Point2LL closest_here = LinearAlg2D::getClosestOnLineSegment(p, point, last);
                int64_t dist = vSize2(p - closest_here);

                if (dist<margin*margin) return true;

                last=point;
            }
        }
        return false;
    };
    
    // Check the results
    for (auto poly:inner_square)
        for (auto point:poly)
            EXPECT_TRUE(checkPointIsPassed(point, MM2INT(1))) << "The corners of this square need an infill line under them so they dont droop down!";
            
    int ctr = 0;
    if (checkPointIsPassed({MM2INT(30),MM2INT(30)}, MM2INT(1))) ctr++;
    if (checkPointIsPassed({MM2INT(8),MM2INT(64)}, MM2INT(1))) ctr++;
    if (checkPointIsPassed({MM2INT(16),MM2INT(42)}, MM2INT(1))) ctr++;
    if (checkPointIsPassed({MM2INT(77),MM2INT(33)}, MM2INT(1))) ctr++;
    if (checkPointIsPassed({MM2INT(12),MM2INT(1)}, MM2INT(1))) ctr++;
    if (checkPointIsPassed({MM2INT(30),MM2INT(30)}, MM2INT(1))) ctr++;
    EXPECT_LE(ctr, 3) << "Randomly selected points should not be supported by sparse infill";
}

} // namespace cura
// NOLINTEND(*-magic-numbers)
