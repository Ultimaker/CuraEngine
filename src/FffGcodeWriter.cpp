// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "FffGcodeWriter.h"

#include <algorithm>
#include <limits> // numeric_limits
#include <list>
#include <memory>
#include <optional>
#include <unordered_set>

#include <range/v3/view/concat.hpp>
#include <range/v3/view/sliding.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>
#include <spdlog/spdlog.h>

#include "Application.h"
#include "ExtruderTrain.h"
#include "FffProcessor.h"
#include "InsetOrderOptimizer.h"
#include "LayerPlan.h"
#include "Slice.h"
#include "WallToolPaths.h"
#include "bridge.h"
#include "communication/Communication.h" //To send layer view data.
#include "infill.h"
#include "progress/Progress.h"
#include "raft.h"
#include "utils/Simplify.h" //Removing micro-segments created by offsetting.
#include "utils/ThreadPool.h"
#include "utils/linearAlg2D.h"
#include "utils/math.h"
#include "utils/orderOptimizer.h"

namespace cura
{

FffGcodeWriter::FffGcodeWriter()
    : max_object_height(0)
    , layer_plan_buffer(gcode)
    , slice_uuid(Application::getInstance().instance_uuid)
{
    for (unsigned int extruder_nr = 0; extruder_nr < MAX_EXTRUDERS; extruder_nr++)
    { // initialize all as max layer_nr, so that they get updated to the lowest layer on which they are used.
        extruder_prime_layer_nr[extruder_nr] = std::numeric_limits<int>::max();
    }
}

void FffGcodeWriter::setTargetStream(std::ostream* stream)
{
    gcode.setOutputStream(stream);
}

double FffGcodeWriter::getTotalFilamentUsed(int extruder_nr)
{
    return gcode.getTotalFilamentUsed(extruder_nr);
}

std::vector<Duration> FffGcodeWriter::getTotalPrintTimePerFeature()
{
    return gcode.getTotalPrintTimePerFeature();
}

bool FffGcodeWriter::setTargetFile(const char* filename)
{
    output_file.open(filename);
    if (output_file.is_open())
    {
        gcode.setOutputStream(&output_file);
        return true;
    }
    return false;
}

void FffGcodeWriter::writeGCode(SliceDataStorage& storage, TimeKeeper& time_keeper)
{
    const size_t start_extruder_nr = getStartExtruder(storage);
    gcode.preSetup(start_extruder_nr);
    gcode.setSliceUUID(slice_uuid);

    Scene& scene = Application::getInstance().current_slice->scene;
    if (scene.current_mesh_group == scene.mesh_groups.begin()) // First mesh group.
    {
        gcode.resetTotalPrintTimeAndFilament();
        gcode.setInitialAndBuildVolumeTemps(start_extruder_nr);
    }

    Application::getInstance().communication->beginGCode();

    setConfigFanSpeedLayerTime();

    setConfigRetractionAndWipe(storage);

    if (scene.current_mesh_group == scene.mesh_groups.begin())
    {
        auto should_prime_extruder = gcode.initializeExtruderTrains(storage, start_extruder_nr);

        if (! should_prime_extruder)
        {
            // set to most negative number so that layer processing never primes this extruder anymore.
            extruder_prime_layer_nr[start_extruder_nr] = std::numeric_limits<int>::min();
        }
    }
    else
    {
        processNextMeshGroupCode(storage);
    }

    size_t total_layers = 0;
    for (std::shared_ptr<SliceMeshStorage>& mesh_ptr : storage.meshes)
    {
        auto& mesh = *mesh_ptr;
        size_t mesh_layer_num = mesh.layers.size();

        // calculation of _actual_ number of layers in loop.
        while (mesh_layer_num > 0 && mesh.layers[mesh_layer_num - 1].getOutlines().empty())
        {
            mesh_layer_num--;
        }

        total_layers = std::max(total_layers, mesh_layer_num);

        setInfillAndSkinAngles(mesh);
    }

    setSupportAngles(storage);

    gcode.writeLayerCountComment(total_layers);

    { // calculate the mesh order for each extruder
        const size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();
        mesh_order_per_extruder.clear(); // Might be not empty in case of sequential printing.
        mesh_order_per_extruder.reserve(extruder_count);
        for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
        {
            mesh_order_per_extruder.push_back(calculateMeshOrder(storage, extruder_nr));
        }
    }
    calculateExtruderOrderPerLayer(storage);
    calculatePrimeLayerPerExtruder(storage);

    if (scene.current_mesh_group->settings.get<bool>("magic_spiralize"))
    {
        findLayerSeamsForSpiralize(storage, total_layers);
    }

    int process_layer_starting_layer_nr = 0;
    const bool has_raft = scene.current_mesh_group->settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT;
    if (has_raft)
    {
        processRaft(storage);
        // process filler layers to fill the airgap with helper object (support etc) so that they stick better to the raft.
        // only process the filler layers if there is anything to print in them.
        for (bool extruder_is_used_in_filler_layers : storage.getExtrudersUsed(-1))
        {
            if (extruder_is_used_in_filler_layers)
            {
                process_layer_starting_layer_nr = -Raft::getFillerLayerCount();
                break;
            }
        }
    }

    run_multiple_producers_ordered_consumer(
        process_layer_starting_layer_nr,
        total_layers,
        [&storage, total_layers, this](int layer_nr)
        {
            return &processLayer(storage, layer_nr, total_layers);
        },
        [this, total_layers](LayerPlan* gcode_layer)
        {
            Progress::messageProgress(Progress::Stage::EXPORT, std::max(LayerIndex{ 0 }, gcode_layer->getLayerNr()) + 1, total_layers);
            layer_plan_buffer.handle(*gcode_layer, gcode);
        });


    layer_plan_buffer.flush();

    Progress::messageProgressStage(Progress::Stage::FINISH, &time_keeper);

    // Store the object height for when we are printing multiple objects, as we need to clear every one of them when moving to the next position.
    max_object_height = std::max(max_object_height, storage.model_max.z);


    constexpr bool force = true;
    gcode.writeRetraction(storage.retraction_wipe_config_per_extruder[gcode.getExtruderNr()].retraction_config, force); // retract after finishing each meshgroup
}

unsigned int FffGcodeWriter::findSpiralizedLayerSeamVertexIndex(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const int layer_nr, const int last_layer_nr)
{
    const SliceLayer& layer = mesh.layers[layer_nr];

    // last_layer_nr will be < 0 until we have processed the first non-empty layer
    if (last_layer_nr < 0)
    {
        // If the user has specified a z-seam location, use the vertex closest to that location for the seam vertex
        // in the first layer that has a part with insets. This allows the user to alter the seam start location which
        // could be useful if the spiralization has a problem with a particular seam path.
        Point seam_pos(0, 0);
        if (mesh.settings.get<EZSeamType>("z_seam_type") == EZSeamType::USER_SPECIFIED)
        {
            seam_pos = mesh.getZSeamHint();
        }
        return PolygonUtils::findClosest(seam_pos, layer.parts[0].spiral_wall[0]).point_idx;
    }
    else
    {
        // note that the code below doesn't assume that last_layer_nr is one less than layer_nr but the print is going
        // to come out pretty weird if that isn't true as it implies that there are empty layers

        ConstPolygonRef last_wall = (*storage.spiralize_wall_outlines[last_layer_nr])[0];
        // Even though this is just one (contiguous) part, the spiralize wall may still be multiple parts if the part is somewhere thinner than 1 line width.
        // This case is so rare that we don't bother with finding the best polygon to start with. Just start with the first polygon (`spiral_wall[0]`).
        ConstPolygonRef wall = layer.parts[0].spiral_wall[0];
        const size_t n_points = wall.size();
        const Point last_wall_seam_vertex = last_wall[storage.spiralize_seam_vertex_indices[last_layer_nr]];

        // seam_vertex_idx is going to be the index of the seam vertex in the current wall polygon
        // initially we choose the vertex that is closest to the seam vertex in the last spiralized layer processed

        int seam_vertex_idx = PolygonUtils::findNearestVert(last_wall_seam_vertex, wall);

        // now we check that the vertex following the seam vertex is to the left of the seam vertex in the last layer
        // and if it isn't, we move forward

        if (vSize(last_wall_seam_vertex - wall[seam_vertex_idx]) >= mesh.settings.get<coord_t>("meshfix_maximum_resolution"))
        {
            // get the inward normal of the last layer seam vertex
            Point last_wall_seam_vertex_inward_normal = PolygonUtils::getVertexInwardNormal(last_wall, storage.spiralize_seam_vertex_indices[last_layer_nr]);

            // create a vector from the normal so that we can then test the vertex following the candidate seam vertex to make sure it is on the correct side
            Point last_wall_seam_vertex_vector = last_wall_seam_vertex + last_wall_seam_vertex_inward_normal;

            // now test the vertex following the candidate seam vertex and if it lies to the left of the vector, it's good to use
            float a = LinearAlg2D::getAngleLeft(last_wall_seam_vertex_vector, last_wall_seam_vertex, wall[(seam_vertex_idx + 1) % n_points]);

            if (a <= 0 || a >= M_PI)
            {
                // the vertex was not on the left of the vector so move the seam vertex on
                seam_vertex_idx = (seam_vertex_idx + 1) % n_points;
                a = LinearAlg2D::getAngleLeft(last_wall_seam_vertex_vector, last_wall_seam_vertex, wall[(seam_vertex_idx + 1) % n_points]);
            }
        }

        return seam_vertex_idx;
    }
}

void FffGcodeWriter::findLayerSeamsForSpiralize(SliceDataStorage& storage, size_t total_layers)
{
    // The spiral has to continue on in an anti-clockwise direction from where the last layer finished, it can't jump backwards

    // we track the seam position for each layer and ensure that the seam position for next layer continues in the right direction

    storage.spiralize_wall_outlines.assign(total_layers, nullptr); // default is no information available
    storage.spiralize_seam_vertex_indices.assign(total_layers, 0);

    int last_layer_nr = -1; // layer number of the last non-empty layer processed (for any extruder or mesh)

    for (unsigned layer_nr = 0; layer_nr < total_layers; ++layer_nr)
    {
        bool done_this_layer = false;

        // iterate through extruders until we find a mesh that has a part with insets
        const std::vector<size_t>& extruder_order = extruder_order_per_layer[layer_nr];
        for (unsigned int extruder_idx = 0; ! done_this_layer && extruder_idx < extruder_order.size(); ++extruder_idx)
        {
            const size_t extruder_nr = extruder_order[extruder_idx];
            // iterate through this extruder's meshes until we find a part with insets
            const std::vector<size_t>& mesh_order = mesh_order_per_extruder[extruder_nr];
            for (unsigned int mesh_idx : mesh_order)
            {
                SliceMeshStorage& mesh = *storage.meshes[mesh_idx];
                // if this mesh has layer data for this layer process it
                if (! done_this_layer && mesh.layers.size() > layer_nr)
                {
                    SliceLayer& layer = mesh.layers[layer_nr];
                    // if the first part in the layer (if any) has insets, process it
                    if (! layer.parts.empty() && ! layer.parts[0].spiral_wall.empty())
                    {
                        // save the seam vertex index for this layer as we need it to determine the seam vertex index for the next layer
                        storage.spiralize_seam_vertex_indices[layer_nr] = findSpiralizedLayerSeamVertexIndex(storage, mesh, layer_nr, last_layer_nr);
                        // save the wall outline for this layer so it can be used in the spiralize interpolation calculation
                        storage.spiralize_wall_outlines[layer_nr] = &layer.parts[0].spiral_wall;
                        last_layer_nr = layer_nr;
                        // ignore any further meshes/extruders for this layer
                        done_this_layer = true;
                    }
                }
            }
        }
    }
}

void FffGcodeWriter::setConfigFanSpeedLayerTime()
{
    for (const ExtruderTrain& train : Application::getInstance().current_slice->scene.extruders)
    {
        fan_speed_layer_time_settings_per_extruder.emplace_back();
        FanSpeedLayerTimeSettings& fan_speed_layer_time_settings = fan_speed_layer_time_settings_per_extruder.back();
        fan_speed_layer_time_settings.cool_min_layer_time = train.settings.get<Duration>("cool_min_layer_time");
        fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max = train.settings.get<Duration>("cool_min_layer_time_fan_speed_max");
        fan_speed_layer_time_settings.cool_fan_speed_0 = train.settings.get<Ratio>("cool_fan_speed_0") * 100.0;
        fan_speed_layer_time_settings.cool_fan_speed_min = train.settings.get<Ratio>("cool_fan_speed_min") * 100.0;
        fan_speed_layer_time_settings.cool_fan_speed_max = train.settings.get<Ratio>("cool_fan_speed_max") * 100.0;
        fan_speed_layer_time_settings.cool_min_speed = train.settings.get<Velocity>("cool_min_speed");
        fan_speed_layer_time_settings.cool_fan_full_layer = train.settings.get<LayerIndex>("cool_fan_full_layer");
        if (! train.settings.get<bool>("cool_fan_enabled"))
        {
            fan_speed_layer_time_settings.cool_fan_speed_0 = 0;
            fan_speed_layer_time_settings.cool_fan_speed_min = 0;
            fan_speed_layer_time_settings.cool_fan_speed_max = 0;
        }
    }
}

static void retractionAndWipeConfigFromSettings(const Settings& settings, RetractionAndWipeConfig* config)
{
    RetractionConfig& retraction_config = config->retraction_config;
    retraction_config.distance = (settings.get<bool>("retraction_enable")) ? settings.get<double>("retraction_amount") : 0; // Retraction distance in mm.
    retraction_config.prime_volume = settings.get<double>("retraction_extra_prime_amount"); // Extra prime volume in mm^3.
    retraction_config.speed = settings.get<Velocity>("retraction_retract_speed");
    retraction_config.primeSpeed = settings.get<Velocity>("retraction_prime_speed");
    retraction_config.zHop = settings.get<coord_t>("retraction_hop");
    retraction_config.retraction_min_travel_distance = settings.get<coord_t>("retraction_min_travel");
    retraction_config.retraction_extrusion_window = settings.get<double>("retraction_extrusion_window"); // Window to count retractions in in mm of extruded filament.
    retraction_config.retraction_count_max = settings.get<size_t>("retraction_count_max");

    config->retraction_hop_after_extruder_switch = settings.get<bool>("retraction_hop_after_extruder_switch");
    config->switch_extruder_extra_prime_amount = settings.get<double>("switch_extruder_extra_prime_amount");
    RetractionConfig& switch_retraction_config = config->extruder_switch_retraction_config;
    switch_retraction_config.distance = settings.get<double>("switch_extruder_retraction_amount"); // Retraction distance in mm.
    switch_retraction_config.prime_volume = 0.0;
    switch_retraction_config.speed = settings.get<Velocity>("switch_extruder_retraction_speed");
    switch_retraction_config.primeSpeed = settings.get<Velocity>("switch_extruder_prime_speed");
    switch_retraction_config.zHop = settings.get<coord_t>("retraction_hop_after_extruder_switch_height");
    switch_retraction_config.retraction_min_travel_distance = 0; // No limitation on travel distance for an extruder switch retract.
    switch_retraction_config.retraction_extrusion_window
        = 99999.9; // So that extruder switch retractions won't affect the retraction buffer (extruded_volume_at_previous_n_retractions).
    switch_retraction_config.retraction_count_max = 9999999; // Extruder switch retraction is never limited.

    WipeScriptConfig& wipe_config = config->wipe_config;

    wipe_config.retraction_enable = settings.get<bool>("wipe_retraction_enable");
    wipe_config.retraction_config.distance = settings.get<double>("wipe_retraction_amount");
    wipe_config.retraction_config.speed = settings.get<Velocity>("wipe_retraction_retract_speed");
    wipe_config.retraction_config.primeSpeed = settings.get<Velocity>("wipe_retraction_prime_speed");
    wipe_config.retraction_config.prime_volume = settings.get<double>("wipe_retraction_extra_prime_amount");
    wipe_config.retraction_config.retraction_min_travel_distance = 0;
    wipe_config.retraction_config.retraction_extrusion_window = std::numeric_limits<double>::max();
    wipe_config.retraction_config.retraction_count_max = std::numeric_limits<size_t>::max();

    wipe_config.pause = settings.get<Duration>("wipe_pause");

    wipe_config.hop_enable = settings.get<bool>("wipe_hop_enable");
    wipe_config.hop_amount = settings.get<coord_t>("wipe_hop_amount");
    wipe_config.hop_speed = settings.get<Velocity>("wipe_hop_speed");

    wipe_config.brush_pos_x = settings.get<coord_t>("wipe_brush_pos_x");
    wipe_config.repeat_count = settings.get<size_t>("wipe_repeat_count");
    wipe_config.move_distance = settings.get<coord_t>("wipe_move_distance");
    wipe_config.move_speed = settings.get<Velocity>("speed_travel");
    wipe_config.max_extrusion_mm3 = settings.get<double>("max_extrusion_before_wipe");
    wipe_config.clean_between_layers = settings.get<bool>("clean_between_layers");
}

void FffGcodeWriter::setConfigRetractionAndWipe(SliceDataStorage& storage)
{
    Scene& scene = Application::getInstance().current_slice->scene;
    for (size_t extruder_index = 0; extruder_index < scene.extruders.size(); extruder_index++)
    {
        ExtruderTrain& train = scene.extruders[extruder_index];
        retractionAndWipeConfigFromSettings(train.settings, &storage.retraction_wipe_config_per_extruder[extruder_index]);
    }
    for (std::shared_ptr<SliceMeshStorage>& mesh : storage.meshes)
    {
        retractionAndWipeConfigFromSettings(mesh->settings, &mesh->retraction_wipe_config);
    }
}

size_t FffGcodeWriter::getStartExtruder(const SliceDataStorage& storage)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const EPlatformAdhesion adhesion_type = mesh_group_settings.get<EPlatformAdhesion>("adhesion_type");
    const int skirt_brim_extruder_nr = mesh_group_settings.get<int>("skirt_brim_extruder_nr");
    const ExtruderTrain* skirt_brim_extruder = (skirt_brim_extruder_nr < 0) ? nullptr : &mesh_group_settings.get<ExtruderTrain&>("skirt_brim_extruder_nr");

    size_t start_extruder_nr;
    if (adhesion_type == EPlatformAdhesion::SKIRT && skirt_brim_extruder
        && (skirt_brim_extruder->settings.get<int>("skirt_line_count") > 0 || skirt_brim_extruder->settings.get<coord_t>("skirt_brim_minimal_length") > 0))
    {
        start_extruder_nr = skirt_brim_extruder->extruder_nr;
    }

    else if (
        (adhesion_type == EPlatformAdhesion::BRIM || mesh_group_settings.get<bool>("prime_tower_brim_enable")) && skirt_brim_extruder
        && (skirt_brim_extruder->settings.get<int>("brim_line_count") > 0 || skirt_brim_extruder->settings.get<coord_t>("skirt_brim_minimal_length") > 0))
    {
        start_extruder_nr = skirt_brim_extruder->extruder_nr;
    }
    else if (adhesion_type == EPlatformAdhesion::RAFT && skirt_brim_extruder)
    {
        start_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("raft_base_extruder_nr").extruder_nr;
    }
    else // No adhesion.
    {
        if (mesh_group_settings.get<bool>("support_enable") && mesh_group_settings.get<bool>("support_brim_enable"))
        {
            start_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr;
        }
        else
        {
            std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
            for (size_t extruder_nr = 0; extruder_nr < extruder_is_used.size(); extruder_nr++)
            {
                start_extruder_nr = extruder_nr;
                if (extruder_is_used[extruder_nr])
                {
                    break;
                }
            }
        }
    }
    assert(start_extruder_nr < Application::getInstance().current_slice->scene.extruders.size() && "start_extruder_nr must be a valid extruder");
    return start_extruder_nr;
}

void FffGcodeWriter::setInfillAndSkinAngles(SliceMeshStorage& mesh)
{
    if (mesh.infill_angles.size() == 0)
    {
        mesh.infill_angles = mesh.settings.get<std::vector<AngleDegrees>>("infill_angles");
        if (mesh.infill_angles.size() == 0)
        {
            // user has not specified any infill angles so use defaults
            const EFillMethod infill_pattern = mesh.settings.get<EFillMethod>("infill_pattern");
            if (infill_pattern == EFillMethod::CROSS || infill_pattern == EFillMethod::CROSS_3D)
            {
                mesh.infill_angles.push_back(22); // put most infill lines in between 45 and 0 degrees
            }
            else
            {
                mesh.infill_angles.push_back(45); // generally all infill patterns use 45 degrees
                if (infill_pattern == EFillMethod::LINES || infill_pattern == EFillMethod::ZIG_ZAG)
                {
                    // lines and zig zag patterns default to also using 135 degrees
                    mesh.infill_angles.push_back(135);
                }
            }
        }
    }

    if (mesh.roofing_angles.size() == 0)
    {
        mesh.roofing_angles = mesh.settings.get<std::vector<AngleDegrees>>("roofing_angles");
        if (mesh.roofing_angles.size() == 0)
        {
            // user has not specified any infill angles so use defaults
            mesh.roofing_angles.push_back(45);
            mesh.roofing_angles.push_back(135);
        }
    }

    if (mesh.skin_angles.size() == 0)
    {
        mesh.skin_angles = mesh.settings.get<std::vector<AngleDegrees>>("skin_angles");
        if (mesh.skin_angles.size() == 0)
        {
            // user has not specified any infill angles so use defaults
            mesh.skin_angles.push_back(45);
            mesh.skin_angles.push_back(135);
        }
    }
}

void FffGcodeWriter::setSupportAngles(SliceDataStorage& storage)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const ExtruderTrain& support_infill_extruder = mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr");
    storage.support.support_infill_angles = support_infill_extruder.settings.get<std::vector<AngleDegrees>>("support_infill_angles");
    if (storage.support.support_infill_angles.empty())
    {
        storage.support.support_infill_angles.push_back(0);
    }

    const ExtruderTrain& support_extruder_nr_layer_0 = mesh_group_settings.get<ExtruderTrain&>("support_extruder_nr_layer_0");
    storage.support.support_infill_angles_layer_0 = support_extruder_nr_layer_0.settings.get<std::vector<AngleDegrees>>("support_infill_angles");
    if (storage.support.support_infill_angles_layer_0.empty())
    {
        storage.support.support_infill_angles_layer_0.push_back(0);
    }

    auto getInterfaceAngles
        = [&storage](const ExtruderTrain& extruder, const std::string& interface_angles_setting, const EFillMethod pattern, const std::string& interface_height_setting)
    {
        std::vector<AngleDegrees> angles = extruder.settings.get<std::vector<AngleDegrees>>(interface_angles_setting);
        if (angles.empty())
        {
            if (pattern == EFillMethod::CONCENTRIC)
            {
                angles.push_back(0); // Concentric has no rotation.
            }
            else if (pattern == EFillMethod::TRIANGLES)
            {
                angles.push_back(90); // Triangular support interface shouldn't alternate every layer.
            }
            else
            {
                for (const auto& mesh : storage.meshes)
                {
                    if (mesh->settings.get<coord_t>(interface_height_setting)
                        >= 2 * Application::getInstance().current_slice->scene.current_mesh_group->settings.get<coord_t>("layer_height"))
                    {
                        // Some roofs are quite thick.
                        // Alternate between the two kinds of diagonal: / and \ .
                        angles.push_back(45);
                        angles.push_back(135);
                    }
                }
                if (angles.empty())
                {
                    angles.push_back(90); // Perpendicular to support lines.
                }
            }
        }
        return angles;
    };

    const ExtruderTrain& roof_extruder = mesh_group_settings.get<ExtruderTrain&>("support_roof_extruder_nr");
    storage.support.support_roof_angles
        = getInterfaceAngles(roof_extruder, "support_roof_angles", roof_extruder.settings.get<EFillMethod>("support_roof_pattern"), "support_roof_height");

    const ExtruderTrain& bottom_extruder = mesh_group_settings.get<ExtruderTrain&>("support_bottom_extruder_nr");
    storage.support.support_bottom_angles
        = getInterfaceAngles(bottom_extruder, "support_bottom_angles", bottom_extruder.settings.get<EFillMethod>("support_bottom_pattern"), "support_bottom_height");
}

void FffGcodeWriter::processNextMeshGroupCode(const SliceDataStorage& storage)
{
    gcode.writeFanCommand(0);
    gcode.setZ(max_object_height + MM2INT(5));

    Application::getInstance().communication->sendCurrentPosition(gcode.getPositionXY());
    gcode.writeTravel(gcode.getPositionXY(), Application::getInstance().current_slice->scene.extruders[gcode.getExtruderNr()].settings.get<Velocity>("speed_travel"));
    Point start_pos(storage.model_min.x, storage.model_min.y);
    gcode.writeTravel(start_pos, Application::getInstance().current_slice->scene.extruders[gcode.getExtruderNr()].settings.get<Velocity>("speed_travel"));

    gcode.processInitialLayerTemperature(storage, gcode.getExtruderNr());
}

void FffGcodeWriter::processRaft(const SliceDataStorage& storage)
{
    Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const size_t base_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("raft_base_extruder_nr").extruder_nr;
    const size_t interface_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("raft_interface_extruder_nr").extruder_nr;
    const size_t surface_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("raft_surface_extruder_nr").extruder_nr;
    const size_t prime_tower_extruder_nr = storage.primeTower.extruder_order.front();

    coord_t z = 0;
    const LayerIndex initial_raft_layer_nr = -Raft::getTotalExtraLayers();
    const Settings& interface_settings = mesh_group_settings.get<ExtruderTrain&>("raft_interface_extruder_nr").settings;
    const size_t num_interface_layers = interface_settings.get<size_t>("raft_interface_layers");
    const Settings& surface_settings = mesh_group_settings.get<ExtruderTrain&>("raft_surface_extruder_nr").settings;
    const size_t num_surface_layers = surface_settings.get<size_t>("raft_surface_layers");

    // some infill config for all lines infill generation below
    constexpr double fill_overlap = 0; // raft line shouldn't be expanded - there is no boundary polygon printed
    constexpr int infill_multiplier = 1; // rafts use single lines
    constexpr int extra_infill_shift = 0;
    constexpr bool fill_gaps = true;

    Polygons raft_polygons; // should remain empty, since we only have the lines pattern for the raft...
    std::optional<Point> last_planned_position = std::optional<Point>();

    unsigned int current_extruder_nr = base_extruder_nr;

    { // raft base layer
        const Settings& base_settings = mesh_group_settings.get<ExtruderTrain&>("raft_base_extruder_nr").settings;
        LayerIndex layer_nr = initial_raft_layer_nr;
        const coord_t layer_height = base_settings.get<coord_t>("raft_base_thickness");
        z += layer_height;
        const coord_t comb_offset = base_settings.get<coord_t>("raft_base_line_spacing");

        std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder_raft_base
            = fan_speed_layer_time_settings_per_extruder; // copy so that we change only the local copy
        for (FanSpeedLayerTimeSettings& fan_speed_layer_time_settings : fan_speed_layer_time_settings_per_extruder_raft_base)
        {
            double regular_fan_speed = base_settings.get<Ratio>("raft_base_fan_speed") * 100.0;
            fan_speed_layer_time_settings.cool_fan_speed_min = regular_fan_speed;
            fan_speed_layer_time_settings.cool_fan_speed_0 = regular_fan_speed; // ignore initial layer fan speed stuff
        }

        const coord_t line_width = base_settings.get<coord_t>("raft_base_line_width");
        const coord_t avoid_distance = base_settings.get<coord_t>("travel_avoid_distance");
        LayerPlan& gcode_layer
            = *new LayerPlan(storage, layer_nr, z, layer_height, base_extruder_nr, fan_speed_layer_time_settings_per_extruder_raft_base, comb_offset, line_width, avoid_distance);
        gcode_layer.setIsInside(true);

        Application::getInstance().communication->sendLayerComplete(layer_nr, z, layer_height);

        Polygons raftLines;
        AngleDegrees fill_angle = (num_surface_layers + num_interface_layers) % 2 ? 45 : 135; // 90 degrees rotated from the interface layer.
        constexpr bool zig_zaggify_infill = false;
        constexpr bool connect_polygons = true; // causes less jerks, so better adhesion
        constexpr bool retract_before_outer_wall = false;
        constexpr coord_t wipe_dist = 0;

        const size_t wall_line_count = base_settings.get<size_t>("raft_base_wall_count");
        const coord_t small_area_width = 0; // A raft never has a small region due to the large horizontal expansion.
        const coord_t line_spacing = base_settings.get<coord_t>("raft_base_line_spacing");
        const coord_t line_spacing_prime_tower = base_settings.get<coord_t>("prime_tower_raft_base_line_spacing");
        const Point& infill_origin = Point();
        constexpr bool skip_stitching = false;
        constexpr bool connected_zigzags = false;
        constexpr bool use_endpieces = true;
        constexpr bool skip_some_zags = false;
        constexpr int zag_skip_count = 0;
        constexpr coord_t pocket_size = 0;
        const coord_t max_resolution = base_settings.get<coord_t>("meshfix_maximum_resolution");
        const coord_t max_deviation = base_settings.get<coord_t>("meshfix_maximum_deviation");

        struct ParameterizedRaftPath
        {
            coord_t line_spacing;
            Polygons outline;
        };

        std::vector<ParameterizedRaftPath> raft_outline_paths;
        raft_outline_paths.emplace_back(ParameterizedRaftPath{ line_spacing, storage.raftOutline });
        if (storage.primeTower.enabled)
        {
            const Polygons& raft_outline_prime_tower = storage.primeTower.getOuterPoly(layer_nr);
            if (line_spacing_prime_tower == line_spacing)
            {
                // Base layer is shared with prime tower base
                raft_outline_paths.front().outline = raft_outline_paths.front().outline.unionPolygons(raft_outline_prime_tower);
            }
            else
            {
                // Prime tower has a different line spacing, print them separately
                raft_outline_paths.front().outline = raft_outline_paths.front().outline.difference(raft_outline_prime_tower);
                raft_outline_paths.emplace_back(ParameterizedRaftPath{ line_spacing_prime_tower, raft_outline_prime_tower });
            }
        }

        for (const ParameterizedRaftPath& raft_outline_path : raft_outline_paths)
        {
            Infill infill_comp(
                EFillMethod::LINES,
                zig_zaggify_infill,
                connect_polygons,
                raft_outline_path.outline,
                gcode_layer.configs_storage.raft_base_config.getLineWidth(),
                raft_outline_path.line_spacing,
                fill_overlap,
                infill_multiplier,
                fill_angle,
                z,
                extra_infill_shift,
                max_resolution,
                max_deviation,
                wall_line_count,
                small_area_width,
                infill_origin,
                skip_stitching,
                fill_gaps,
                connected_zigzags,
                use_endpieces,
                skip_some_zags,
                zag_skip_count,
                pocket_size);
            std::vector<VariableWidthLines> raft_paths;
            infill_comp.generate(raft_paths, raft_polygons, raftLines, base_settings, layer_nr, SectionType::ADHESION);
            if (! raft_paths.empty())
            {
                const GCodePathConfig& config = gcode_layer.configs_storage.raft_base_config;
                const ZSeamConfig z_seam_config(EZSeamType::SHORTEST, gcode_layer.getLastPlannedPositionOrStartingPosition(), EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE, false);
                InsetOrderOptimizer wall_orderer(
                    *this,
                    storage,
                    gcode_layer,
                    base_settings,
                    base_extruder_nr,
                    config,
                    config,
                    config,
                    config,
                    retract_before_outer_wall,
                    wipe_dist,
                    wipe_dist,
                    base_extruder_nr,
                    base_extruder_nr,
                    z_seam_config,
                    raft_paths);
                wall_orderer.addToLayer();
            }
            gcode_layer.addLinesByOptimizer(raftLines, gcode_layer.configs_storage.raft_base_config, SpaceFillType::Lines);

            raft_polygons.clear();
            raftLines.clear();
        }

        layer_plan_buffer.handle(gcode_layer, gcode);
        last_planned_position = gcode_layer.getLastPlannedPositionOrStartingPosition();
    }

    const coord_t interface_layer_height = interface_settings.get<coord_t>("raft_interface_thickness");
    const coord_t interface_line_spacing = interface_settings.get<coord_t>("raft_interface_line_spacing");
    const Ratio interface_fan_speed = interface_settings.get<Ratio>("raft_interface_fan_speed");
    const coord_t interface_line_width = interface_settings.get<coord_t>("raft_interface_line_width");
    const coord_t interface_avoid_distance = interface_settings.get<coord_t>("travel_avoid_distance");
    const coord_t interface_max_resolution = interface_settings.get<coord_t>("meshfix_maximum_resolution");
    const coord_t interface_max_deviation = interface_settings.get<coord_t>("meshfix_maximum_deviation");

    for (LayerIndex raft_interface_layer = 1; static_cast<size_t>(raft_interface_layer) <= num_interface_layers; ++raft_interface_layer)
    { // raft interface layer
        bool prime_tower_added_on_this_layer = ! storage.primeTower.enabled;
        const LayerIndex layer_nr = initial_raft_layer_nr + raft_interface_layer;
        z += interface_layer_height;

        std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder_raft_interface
            = fan_speed_layer_time_settings_per_extruder; // copy so that we change only the local copy
        for (FanSpeedLayerTimeSettings& fan_speed_layer_time_settings : fan_speed_layer_time_settings_per_extruder_raft_interface)
        {
            const double regular_fan_speed = interface_fan_speed * 100.0;
            fan_speed_layer_time_settings.cool_fan_speed_min = regular_fan_speed;
            fan_speed_layer_time_settings.cool_fan_speed_0 = regular_fan_speed; // ignore initial layer fan speed stuff
        }

        const coord_t comb_offset = interface_line_spacing;
        LayerPlan& gcode_layer = *new LayerPlan(
            storage,
            layer_nr,
            z,
            interface_layer_height,
            current_extruder_nr,
            fan_speed_layer_time_settings_per_extruder_raft_interface,
            comb_offset,
            interface_line_width,
            interface_avoid_distance);

        if (! prime_tower_added_on_this_layer && current_extruder_nr == prime_tower_extruder_nr)
        {
            addPrimeTower(storage, gcode_layer, current_extruder_nr);
            prime_tower_added_on_this_layer = true;
        }

        gcode_layer.setIsInside(true);
        if (interface_extruder_nr != current_extruder_nr)
        {
            setExtruder_addPrime(storage, gcode_layer, interface_extruder_nr);
            current_extruder_nr = interface_extruder_nr;
        }

        Application::getInstance().communication->sendLayerComplete(layer_nr, z, interface_layer_height);

        Polygons raft_outline_path;
        const coord_t small_offset = gcode_layer.configs_storage.raft_interface_config.getLineWidth()
                                   / 2; // Do this manually because of micron-movement created in corners when insetting a polygon that was offset with round joint type.
        raft_outline_path = storage.raftOutline.offset(-small_offset);
        raft_outline_path = Simplify(interface_settings).polygon(raft_outline_path); // Remove those micron-movements.
        const coord_t infill_outline_width = gcode_layer.configs_storage.raft_interface_config.getLineWidth();
        Polygons raft_lines;
        AngleDegrees fill_angle = (num_surface_layers + num_interface_layers - raft_interface_layer) % 2 ? 45 : 135; // 90 degrees rotated from the first top layer.
        constexpr bool zig_zaggify_infill = true;
        constexpr bool connect_polygons = true; // why not?

        constexpr int wall_line_count = 0;
        const coord_t small_area_width = 0; // A raft never has a small region due to the large horizontal expansion.
        const Point infill_origin = Point();
        constexpr bool skip_stitching = false;
        constexpr bool connected_zigzags = false;
        constexpr bool use_endpieces = true;
        constexpr bool skip_some_zags = false;
        constexpr int zag_skip_count = 0;
        constexpr coord_t pocket_size = 0;

        if (storage.primeTower.enabled)
        {
            // Interface layer excludes prime tower base
            raft_outline_path = raft_outline_path.difference(storage.primeTower.getOuterPoly(layer_nr));
        }

        Infill infill_comp(
            EFillMethod::ZIG_ZAG,
            zig_zaggify_infill,
            connect_polygons,
            raft_outline_path,
            infill_outline_width,
            interface_line_spacing,
            fill_overlap,
            infill_multiplier,
            fill_angle,
            z,
            extra_infill_shift,
            interface_max_resolution,
            interface_max_deviation,
            wall_line_count,
            small_area_width,
            infill_origin,
            skip_stitching,
            fill_gaps,
            connected_zigzags,
            use_endpieces,
            skip_some_zags,
            zag_skip_count,
            pocket_size);
        std::vector<VariableWidthLines> raft_paths; // Should remain empty, since we have no walls.
        infill_comp.generate(raft_paths, raft_polygons, raft_lines, interface_settings, layer_nr, SectionType::ADHESION);
        gcode_layer.addLinesByOptimizer(raft_lines, gcode_layer.configs_storage.raft_interface_config, SpaceFillType::Lines, false, 0, 1.0, last_planned_position);

        raft_polygons.clear();
        raft_lines.clear();

        if (! prime_tower_added_on_this_layer)
        {
            setExtruder_addPrime(storage, gcode_layer, prime_tower_extruder_nr);
            current_extruder_nr = prime_tower_extruder_nr;
        }

        layer_plan_buffer.handle(gcode_layer, gcode);
        last_planned_position = gcode_layer.getLastPlannedPositionOrStartingPosition();
    }

    const coord_t surface_layer_height = surface_settings.get<coord_t>("raft_surface_thickness");
    const coord_t surface_line_spacing = surface_settings.get<coord_t>("raft_surface_line_spacing");
    const coord_t surface_max_resolution = surface_settings.get<coord_t>("meshfix_maximum_resolution");
    const coord_t surface_max_deviation = surface_settings.get<coord_t>("meshfix_maximum_deviation");
    const coord_t surface_line_width = surface_settings.get<coord_t>("raft_surface_line_width");
    const coord_t surface_avoid_distance = surface_settings.get<coord_t>("travel_avoid_distance");
    const Ratio surface_fan_speed = surface_settings.get<Ratio>("raft_surface_fan_speed");

    for (LayerIndex raft_surface_layer = 1; static_cast<size_t>(raft_surface_layer) <= num_surface_layers; raft_surface_layer++)
    { // raft surface layers
        bool prime_tower_added_on_this_layer = ! storage.primeTower.enabled;
        const LayerIndex layer_nr = initial_raft_layer_nr + 1 + num_interface_layers + raft_surface_layer - 1; // +1: 1 base layer
        z += surface_layer_height;

        std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder_raft_surface
            = fan_speed_layer_time_settings_per_extruder; // copy so that we change only the local copy
        for (FanSpeedLayerTimeSettings& fan_speed_layer_time_settings : fan_speed_layer_time_settings_per_extruder_raft_surface)
        {
            const double regular_fan_speed = surface_fan_speed * 100.0;
            fan_speed_layer_time_settings.cool_fan_speed_min = regular_fan_speed;
            fan_speed_layer_time_settings.cool_fan_speed_0 = regular_fan_speed; // ignore initial layer fan speed stuff
        }

        const coord_t comb_offset = surface_line_spacing;
        LayerPlan& gcode_layer = *new LayerPlan(
            storage,
            layer_nr,
            z,
            surface_layer_height,
            current_extruder_nr,
            fan_speed_layer_time_settings_per_extruder_raft_surface,
            comb_offset,
            surface_line_width,
            surface_avoid_distance);

        if (! prime_tower_added_on_this_layer && current_extruder_nr == prime_tower_extruder_nr)
        {
            addPrimeTower(storage, gcode_layer, current_extruder_nr);
            prime_tower_added_on_this_layer = true;
        }

        gcode_layer.setIsInside(true);

        // make sure that we are using the correct extruder to print raft
        if (current_extruder_nr != surface_extruder_nr)
        {
            setExtruder_addPrime(storage, gcode_layer, surface_extruder_nr);
            current_extruder_nr = surface_extruder_nr;
        }
        Application::getInstance().communication->sendLayerComplete(layer_nr, z, surface_layer_height);

        Polygons raft_outline_path;
        const coord_t small_offset = gcode_layer.configs_storage.raft_interface_config.getLineWidth()
                                   / 2; // Do this manually because of micron-movement created in corners when insetting a polygon that was offset with round joint type.
        raft_outline_path = storage.raftOutline.offset(-small_offset);
        raft_outline_path = Simplify(interface_settings).polygon(raft_outline_path); // Remove those micron-movements.
        const coord_t infill_outline_width = gcode_layer.configs_storage.raft_interface_config.getLineWidth();
        Polygons raft_lines;
        AngleDegrees fill_angle
            = (num_surface_layers - raft_surface_layer) % 2 ? 45 : 135; // Alternate between -45 and +45 degrees, ending up 90 degrees rotated from the default skin angle.
        constexpr bool zig_zaggify_infill = true;

        constexpr size_t wall_line_count = 0;
        const coord_t small_area_width = 0; // A raft never has a small region due to the large horizontal expansion.
        const Point& infill_origin = Point();
        constexpr bool skip_stitching = false;
        constexpr bool connected_zigzags = false;
        constexpr bool connect_polygons = false; // midway connections between polygons can make the surface less smooth
        constexpr bool use_endpieces = true;
        constexpr bool skip_some_zags = false;
        constexpr size_t zag_skip_count = 0;
        constexpr coord_t pocket_size = 0;

        if (storage.primeTower.enabled)
        {
            // Surface layers exclude prime tower base
            raft_outline_path = raft_outline_path.difference(storage.primeTower.getOuterPoly(layer_nr));
        }

        Infill infill_comp(
            EFillMethod::ZIG_ZAG,
            zig_zaggify_infill,
            connect_polygons,
            raft_outline_path,
            infill_outline_width,
            surface_line_spacing,
            fill_overlap,
            infill_multiplier,
            fill_angle,
            z,
            extra_infill_shift,
            surface_max_resolution,
            surface_max_deviation,
            wall_line_count,
            small_area_width,
            infill_origin,
            skip_stitching,
            fill_gaps,
            connected_zigzags,
            use_endpieces,
            skip_some_zags,
            zag_skip_count,
            pocket_size);
        std::vector<VariableWidthLines> raft_paths; // Should remain empty, since we have no walls.
        infill_comp.generate(raft_paths, raft_polygons, raft_lines, surface_settings, layer_nr, SectionType::ADHESION);
        gcode_layer.addLinesByOptimizer(raft_lines, gcode_layer.configs_storage.raft_surface_config, SpaceFillType::Lines, false, 0, 1.0, last_planned_position);

        raft_polygons.clear();
        raft_lines.clear();

        if (! prime_tower_added_on_this_layer)
        {
            setExtruder_addPrime(storage, gcode_layer, prime_tower_extruder_nr);
            current_extruder_nr = prime_tower_extruder_nr;
        }

        layer_plan_buffer.handle(gcode_layer, gcode);
    }
}

LayerPlan& FffGcodeWriter::processLayer(const SliceDataStorage& storage, LayerIndex layer_nr, const size_t total_layers) const
{
    spdlog::debug("GcodeWriter processing layer {} of {}", layer_nr, total_layers);

    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    coord_t layer_thickness = mesh_group_settings.get<coord_t>("layer_height");
    coord_t z;
    bool include_helper_parts = true;
    if (layer_nr < 0)
    {
#ifdef DEBUG
        assert(mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT && "negative layer_number means post-raft, pre-model layer!");
#endif // DEBUG
        const int filler_layer_count = Raft::getFillerLayerCount();
        layer_thickness = Raft::getFillerLayerHeight();
        z = Raft::getTotalThickness() + (filler_layer_count + layer_nr + 1) * layer_thickness;
    }
    else
    {
        z = storage.meshes[0]->layers[layer_nr].printZ; // stub default
        // find printZ of first actual printed mesh
        for (const std::shared_ptr<SliceMeshStorage>& mesh_ptr : storage.meshes)
        {
            const auto& mesh = *mesh_ptr;
            if (layer_nr >= static_cast<int>(mesh.layers.size()) || mesh.settings.get<bool>("support_mesh") || mesh.settings.get<bool>("anti_overhang_mesh")
                || mesh.settings.get<bool>("cutting_mesh") || mesh.settings.get<bool>("infill_mesh"))
            {
                continue;
            }
            z = mesh.layers[layer_nr].printZ;
            layer_thickness = mesh.layers[layer_nr].thickness;
            break;
        }

        if (layer_nr < 0 && mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT)
        {
            include_helper_parts = false;
        }
    }

    const Scene& scene = Application::getInstance().current_slice->scene;

    coord_t avoid_distance = 0; // minimal avoid distance is zero
    const std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
    for (size_t extruder_nr = 0; extruder_nr < scene.extruders.size(); extruder_nr++)
    {
        if (extruder_is_used[extruder_nr])
        {
            const ExtruderTrain& extruder = scene.extruders[extruder_nr];

            if (extruder.settings.get<bool>("travel_avoid_other_parts"))
            {
                avoid_distance = std::max(avoid_distance, extruder.settings.get<coord_t>("travel_avoid_distance"));
            }
        }
    }

    coord_t max_inner_wall_width = 0;
    for (const std::shared_ptr<SliceMeshStorage>& mesh_ptr : storage.meshes)
    {
        const auto& mesh = *mesh_ptr;
        coord_t mesh_inner_wall_width = mesh.settings.get<coord_t>((mesh.settings.get<size_t>("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0");
        if (layer_nr == 0)
        {
            const ExtruderTrain& train = mesh.settings.get<ExtruderTrain&>((mesh.settings.get<size_t>("wall_line_count") > 1) ? "wall_0_extruder_nr" : "wall_x_extruder_nr");
            mesh_inner_wall_width *= train.settings.get<Ratio>("initial_layer_line_width_factor");
        }
        max_inner_wall_width = std::max(max_inner_wall_width, mesh_inner_wall_width);
    }
    const coord_t comb_offset_from_outlines = max_inner_wall_width * 2;

    assert(
        static_cast<LayerIndex>(extruder_order_per_layer_negative_layers.size()) + layer_nr >= 0 && "Layer numbers shouldn't get more negative than there are raft/filler layers");
    const std::vector<size_t>& extruder_order
        = (layer_nr < 0) ? extruder_order_per_layer_negative_layers[extruder_order_per_layer_negative_layers.size() + layer_nr] : extruder_order_per_layer[layer_nr];

    const coord_t first_outer_wall_line_width = scene.extruders[extruder_order.front()].settings.get<coord_t>("wall_line_width_0");
    LayerPlan& gcode_layer = *new LayerPlan(
        storage,
        layer_nr,
        z,
        layer_thickness,
        extruder_order.front(),
        fan_speed_layer_time_settings_per_extruder,
        comb_offset_from_outlines,
        first_outer_wall_line_width,
        avoid_distance);

    if (include_helper_parts)
    {
        // process the skirt or the brim of the starting extruder.
        auto extruder_nr = gcode_layer.getExtruder();
        if (storage.skirt_brim[extruder_nr].size() > 0)
        {
            processSkirtBrim(storage, gcode_layer, extruder_nr, layer_nr);
        }

        // handle shield(s) first in a layer so that chances are higher that the other nozzle is wiped (for the ooze shield)
        processOozeShield(storage, gcode_layer);
        processDraftShield(storage, gcode_layer);
    }

    const size_t support_roof_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("support_roof_extruder_nr").extruder_nr;
    const size_t support_bottom_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("support_bottom_extruder_nr").extruder_nr;
    const size_t support_infill_extruder_nr = (layer_nr <= 0) ? mesh_group_settings.get<ExtruderTrain&>("support_extruder_nr_layer_0").extruder_nr
                                                              : mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr;

    for (const size_t& extruder_nr : extruder_order)
    {
        // Everytime you start with a new extruder you want to add a prime tower, unless:
        //  - prime tower is disabled (setExtruder_addPrime takes care of this)
        //  - this is the first (and not the only!) extruder in this layer. Since the previous
        //    layer always ends with this extruder. If the first extruder is the only extruder,
        //    the prime tower needs to be added anyways, in order to support the prime tower if
        //    later in the print a prime tower is needed.
        //  - prime tower is already printed this layer (only applicable for more than 2 extruders).
        //    The setExtruder_addPrime takes care of this.
        if (extruder_nr != extruder_order.front() || (extruder_order.size() == 1 && layer_nr >= 0) || extruder_nr == 0)
        {
            setExtruder_addPrime(storage, gcode_layer, extruder_nr);
        }
        if (include_helper_parts && (extruder_nr == support_infill_extruder_nr || extruder_nr == support_roof_extruder_nr || extruder_nr == support_bottom_extruder_nr))
        {
            addSupportToGCode(storage, gcode_layer, extruder_nr);
        }
        if (layer_nr >= 0)
        {
            const std::vector<size_t>& mesh_order = mesh_order_per_extruder[extruder_nr];
            for (size_t mesh_idx : mesh_order)
            {
                const std::shared_ptr<SliceMeshStorage>& mesh = storage.meshes[mesh_idx];
                const MeshPathConfigs& mesh_config = gcode_layer.configs_storage.mesh_configs[mesh_idx];
                if (mesh->settings.get<ESurfaceMode>("magic_mesh_surface_mode") == ESurfaceMode::SURFACE
                    && extruder_nr
                           == mesh->settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr // mesh surface mode should always only be printed with the outer wall extruder!
                )
                {
                    addMeshLayerToGCode_meshSurfaceMode(storage, *mesh, mesh_config, gcode_layer);
                }
                else
                {
                    addMeshLayerToGCode(storage, mesh, extruder_nr, mesh_config, gcode_layer);
                }
            }
        }
        // Always print a prime tower before switching extruder. Unless:
        //  - The prime tower is already printed this layer (setExtruder_addPrime takes care of this).
        //  - this is the last extruder of the layer, since the next layer will start with the same extruder.
        if (extruder_nr != extruder_order.back() && layer_nr >= 0)
        {
            setExtruder_addPrime(storage, gcode_layer, extruder_nr);
        }
    }

    gcode_layer.applyModifyPlugin();
    gcode_layer.applyBackPressureCompensation();
    return gcode_layer;
}

bool FffGcodeWriter::getExtruderNeedPrimeBlobDuringFirstLayer(const SliceDataStorage& storage, const size_t extruder_nr) const
{
    auto need_prime_blob = gcode.needPrimeBlob();

    // check the settings if the prime blob is disabled
    if (need_prime_blob)
    {
        const bool is_extruder_used_overall = storage.getExtrudersUsed()[extruder_nr];
        const bool extruder_prime_blob_enabled = storage.getExtruderPrimeBlobEnabled(extruder_nr);

        need_prime_blob = is_extruder_used_overall && extruder_prime_blob_enabled;
    }

    return need_prime_blob;
}

void FffGcodeWriter::processSkirtBrim(const SliceDataStorage& storage, LayerPlan& gcode_layer, unsigned int extruder_nr, LayerIndex layer_nr) const
{
    const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];
    const int skirt_height = train.settings.get<int>("skirt_height");
    const bool is_skirt = train.settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::SKIRT;
    // only create a multilayer SkirtBrim for a skirt for the height of skirt_height
    if (layer_nr != 0 && (layer_nr >= skirt_height || ! is_skirt))
    {
        return;
    }
    if (gcode_layer.getSkirtBrimIsPlanned(extruder_nr))
    {
        return;
    }
    gcode_layer.setSkirtBrimIsPlanned(extruder_nr);

    const auto& original_skirt_brim = storage.skirt_brim[extruder_nr];
    if (original_skirt_brim.size() == 0)
    {
        return;
    }

    // Start brim close to the prime location
    Point start_close_to;
    if (train.settings.get<bool>("prime_blob_enable"))
    {
        const auto prime_pos_is_abs = train.settings.get<bool>("extruder_prime_pos_abs");
        const auto prime_pos = Point(train.settings.get<coord_t>("extruder_prime_pos_x"), train.settings.get<coord_t>("extruder_prime_pos_y"));
        start_close_to = prime_pos_is_abs ? prime_pos : gcode_layer.getLastPlannedPositionOrStartingPosition() + prime_pos;
    }
    else
    {
        start_close_to = gcode_layer.getLastPlannedPositionOrStartingPosition();
    }

    // figure out order requirements
    struct BrimLineReference
    {
        const size_t inset_idx;
        ConstPolygonPointer poly;
    };

    size_t total_line_count = 0;
    for (const SkirtBrimLine& line : storage.skirt_brim[extruder_nr])
    {
        total_line_count += line.closed_polygons.size();
        total_line_count += line.open_polylines.size();
    }
    Polygons all_brim_lines;


    all_brim_lines.reserve(total_line_count);

    const coord_t line_w = train.settings.get<coord_t>("skirt_brim_line_width") * train.settings.get<Ratio>("initial_layer_line_width_factor");
    const coord_t searching_radius = line_w * 2;
    using GridT = SparsePointGridInclusive<BrimLineReference>;
    GridT grid(searching_radius);

    for (size_t inset_idx = 0; inset_idx < storage.skirt_brim[extruder_nr].size(); inset_idx++)
    {
        const auto& offset = storage.skirt_brim[extruder_nr][inset_idx];
        const auto closed_polygons_open_polylines = { offset.closed_polygons, offset.open_polylines };
        const auto closed_open = { true, false };
        for (const auto [polygon, closed] : ranges::views::zip(closed_polygons_open_polylines, closed_open))
        {
            for (ConstPolygonRef line : polygon)
            {
                if (line.size() <= 1)
                {
                    continue;
                }
                all_brim_lines.emplace_back(line);
                if (closed)
                {
                    // add closing segment
                    all_brim_lines.back().add(line.front());
                }
                ConstPolygonPointer pp(all_brim_lines.back());
                for (Point p : line)
                {
                    grid.insert(p, BrimLineReference{ inset_idx, pp });
                }
            }
        }
    }

    const auto smart_brim_ordering = train.settings.get<bool>("brim_smart_ordering") && train.settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::BRIM;
    std::unordered_multimap<ConstPolygonPointer, ConstPolygonPointer> order_requirements;
    for (const std::pair<SquareGrid::GridPoint, SparsePointGridInclusiveImpl::SparsePointGridInclusiveElem<BrimLineReference>>& p : grid)
    {
        const BrimLineReference& here = p.second.val;
        Point loc_here = p.second.point;
        std::vector<BrimLineReference> nearby_verts = grid.getNearbyVals(loc_here, searching_radius);
        for (const BrimLineReference& nearby : nearby_verts)
        {
            if (nearby.poly == here.poly || nearby.inset_idx == here.inset_idx)
            {
                continue;
            }

            const BrimLineReference& lower_inset = here.inset_idx < nearby.inset_idx ? here : nearby;
            const BrimLineReference& higher_inset = here.inset_idx < nearby.inset_idx ? nearby : here;

            if (smart_brim_ordering)
            {
                // apply "smart brim ordering" by swapping innermost and second innermost brim lines
                // The "order requirements" tree should look like: n -> n-1 -> ... -> 3 -> 2 -> 0 -> 1
                if (lower_inset.inset_idx == 0 && higher_inset.inset_idx == 1)
                {
                    order_requirements.emplace(lower_inset.poly, higher_inset.poly);
                    continue;
                }
                else if (lower_inset.inset_idx == 0 && higher_inset.inset_idx == 2)
                {
                    order_requirements.emplace(higher_inset.poly, lower_inset.poly);
                    continue;
                }
                else if (lower_inset.inset_idx == 1 && higher_inset.inset_idx == 2)
                {
                    // not directly adjacent
                    continue;
                }
            }

            if (higher_inset.inset_idx > lower_inset.inset_idx + 1)
            {
                // not directly adjacent
                continue;
            }

            order_requirements.emplace(higher_inset.poly, lower_inset.poly);
        }
    }
    assert(all_brim_lines.size() == total_line_count); // Otherwise pointers would have gotten invalidated

    const bool enable_travel_optimization = true; // Use the combing outline while deciding in which order to print the lines. Can't hurt for only one layer.
    const coord_t wipe_dist = 0u;
    const Ratio flow_ratio = 1.0;
    const double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT;
    const bool reverse_print_direction = false;

    if (! all_brim_lines.empty())
    {
        // For layer_nr != 0 add only the innermost brim line (which is only the case if skirt_height > 1)
        Polygons inner_brim_line;
        inner_brim_line.add(all_brim_lines[0]);

        gcode_layer.addLinesByOptimizer(
            layer_nr == 0 ? all_brim_lines : inner_brim_line,
            gcode_layer.configs_storage.skirt_brim_config_per_extruder[extruder_nr],
            SpaceFillType::PolyLines,
            enable_travel_optimization,
            wipe_dist,
            flow_ratio,
            start_close_to,
            fan_speed,
            reverse_print_direction,
            order_requirements);
    }


    // Add the support brim after the skirt_brim to gcode_layer
    // Support brim is only added in layer 0
    // For support brim we don't care about the order, because support doesn't need to be accurate.
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    if ((layer_nr == 0) && (extruder_nr == mesh_group_settings.get<ExtruderTrain&>("support_extruder_nr_layer_0").extruder_nr))
    {
        total_line_count += storage.support_brim.size();
        Polygons support_brim_lines = storage.support_brim;
        support_brim_lines.toPolylines();
        gcode_layer.addLinesByOptimizer(
            support_brim_lines,
            gcode_layer.configs_storage.skirt_brim_config_per_extruder[extruder_nr],
            SpaceFillType::PolyLines,
            enable_travel_optimization,
            wipe_dist,
            flow_ratio,
            start_close_to,
            fan_speed,
            reverse_print_direction,
            order_requirements = {});
    }
}

void FffGcodeWriter::processOozeShield(const SliceDataStorage& storage, LayerPlan& gcode_layer) const
{
    LayerIndex layer_nr = std::max(LayerIndex{ 0 }, gcode_layer.getLayerNr());
    if (layer_nr == 0 && Application::getInstance().current_slice->scene.current_mesh_group->settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::BRIM)
    {
        return; // ooze shield already generated by brim
    }
    if (storage.oozeShield.size() > 0 && layer_nr < storage.oozeShield.size())
    {
        gcode_layer.addPolygonsByOptimizer(storage.oozeShield[layer_nr], gcode_layer.configs_storage.skirt_brim_config_per_extruder[0]);
    }
}

void FffGcodeWriter::processDraftShield(const SliceDataStorage& storage, LayerPlan& gcode_layer) const
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const LayerIndex layer_nr = std::max(LayerIndex{ 0 }, gcode_layer.getLayerNr());
    if (storage.draft_protection_shield.size() == 0)
    {
        return;
    }
    if (! mesh_group_settings.get<bool>("draft_shield_enabled"))
    {
        return;
    }
    if (layer_nr == 0 && Application::getInstance().current_slice->scene.current_mesh_group->settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::BRIM)
    {
        return; // draft shield already generated by brim
    }

    if (mesh_group_settings.get<DraftShieldHeightLimitation>("draft_shield_height_limitation") == DraftShieldHeightLimitation::LIMITED)
    {
        const coord_t draft_shield_height = mesh_group_settings.get<coord_t>("draft_shield_height");
        const coord_t layer_height_0 = mesh_group_settings.get<coord_t>("layer_height_0");
        const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
        const LayerIndex max_screen_layer = (draft_shield_height - layer_height_0) / layer_height + 1;
        if (layer_nr > max_screen_layer)
        {
            return;
        }
    }

    gcode_layer.addPolygonsByOptimizer(storage.draft_protection_shield, gcode_layer.configs_storage.skirt_brim_config_per_extruder[0]);
}

void FffGcodeWriter::calculateExtruderOrderPerLayer(const SliceDataStorage& storage)
{
    size_t last_extruder;
    // set the initial extruder of this meshgroup
    Scene& scene = Application::getInstance().current_slice->scene;
    if (scene.current_mesh_group == scene.mesh_groups.begin())
    { // first meshgroup
        last_extruder = getStartExtruder(storage);
    }
    else
    {
        last_extruder = gcode.getExtruderNr();
    }
    for (LayerIndex layer_nr = -Raft::getTotalExtraLayers(); layer_nr < static_cast<LayerIndex>(storage.print_layer_count); layer_nr++)
    {
        std::vector<std::vector<size_t>>& extruder_order_per_layer_here = (layer_nr < 0) ? extruder_order_per_layer_negative_layers : extruder_order_per_layer;
        extruder_order_per_layer_here.push_back(getUsedExtrudersOnLayerExcludingStartingExtruder(storage, last_extruder, layer_nr));
        last_extruder = extruder_order_per_layer_here.back().back();
    }
}

void FffGcodeWriter::calculatePrimeLayerPerExtruder(const SliceDataStorage& storage)
{
    for (LayerIndex layer_nr = -Raft::getTotalExtraLayers(); layer_nr < static_cast<LayerIndex>(storage.print_layer_count); ++layer_nr)
    {
        const std::vector<bool> used_extruders = storage.getExtrudersUsed(layer_nr);
        for (size_t extruder_nr = 0; extruder_nr < used_extruders.size(); ++extruder_nr)
        {
            if (used_extruders[extruder_nr])
            {
                extruder_prime_layer_nr[extruder_nr] = std::min(extruder_prime_layer_nr[extruder_nr], layer_nr);
            }
        }
    }
}

std::vector<size_t> FffGcodeWriter::getUsedExtrudersOnLayerExcludingStartingExtruder(const SliceDataStorage& storage, const size_t start_extruder, const LayerIndex& layer_nr) const
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();
    assert(static_cast<int>(extruder_count) > 0);
    std::vector<size_t> ret;
    ret.push_back(start_extruder);
    std::vector<bool> extruder_is_used_on_this_layer = storage.getExtrudersUsed(layer_nr);

    // The outermost prime tower extruder is always used if there is a prime tower, apart on layers with negative index (e.g. for the raft)
    if (mesh_group_settings.get<bool>("prime_tower_enable") && /*layer_nr >= 0 &&*/ layer_nr <= storage.max_print_height_second_to_last_extruder)
    {
        extruder_is_used_on_this_layer[storage.primeTower.extruder_order[0]] = true;
    }

    // check if we are on the first layer
    if ((mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT && layer_nr == -static_cast<LayerIndex>(Raft::getTotalExtraLayers()))
        || (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::RAFT && layer_nr == 0))
    {
        // check if we need prime blob on the first layer
        for (size_t used_idx = 0; used_idx < extruder_is_used_on_this_layer.size(); used_idx++)
        {
            if (getExtruderNeedPrimeBlobDuringFirstLayer(storage, used_idx))
            {
                extruder_is_used_on_this_layer[used_idx] = true;
            }
        }
    }

    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        if (extruder_nr == start_extruder)
        { // skip the current extruder, it's the one we started out planning
            continue;
        }
        if (! extruder_is_used_on_this_layer[extruder_nr])
        {
            continue;
        }
        ret.push_back(extruder_nr);
    }
    assert(ret.size() <= (size_t)extruder_count && "Not more extruders may be planned in a layer than there are extruders!");
    return ret;
}

std::vector<size_t> FffGcodeWriter::calculateMeshOrder(const SliceDataStorage& storage, const size_t extruder_nr) const
{
    OrderOptimizer<size_t> mesh_idx_order_optimizer;

    std::vector<MeshGroup>::iterator mesh_group = Application::getInstance().current_slice->scene.current_mesh_group;
    for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        const SliceMeshStorage& mesh = *storage.meshes[mesh_idx];
        if (mesh.getExtruderIsUsed(extruder_nr))
        {
            const Mesh& mesh_data = mesh_group->meshes[mesh_idx];
            const Point3 middle = (mesh_data.getAABB().min + mesh_data.getAABB().max) / 2;
            mesh_idx_order_optimizer.addItem(Point(middle.x, middle.y), mesh_idx);
        }
    }
    const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];
    const Point layer_start_position(train.settings.get<coord_t>("layer_start_x"), train.settings.get<coord_t>("layer_start_y"));
    std::list<size_t> mesh_indices_order = mesh_idx_order_optimizer.optimize(layer_start_position);

    std::vector<size_t> ret;
    ret.reserve(mesh_indices_order.size());

    for (size_t i : mesh_indices_order)
    {
        const size_t mesh_idx = mesh_idx_order_optimizer.items[i].second;
        ret.push_back(mesh_idx);
    }
    return ret;
}

void FffGcodeWriter::addMeshLayerToGCode_meshSurfaceMode(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const MeshPathConfigs& mesh_config, LayerPlan& gcode_layer)
    const
{
    if (gcode_layer.getLayerNr() > mesh.layer_nr_max_filled_layer)
    {
        return;
    }

    if (mesh.settings.get<bool>("anti_overhang_mesh") || mesh.settings.get<bool>("support_mesh"))
    {
        return;
    }

    const SliceLayer* layer = &mesh.layers[gcode_layer.getLayerNr()];


    Polygons polygons;
    for (const SliceLayerPart& part : layer->parts)
    {
        polygons.add(part.outline);
    }

    polygons = Simplify(mesh.settings).polygon(polygons);

    ZSeamConfig z_seam_config(
        mesh.settings.get<EZSeamType>("z_seam_type"),
        mesh.getZSeamHint(),
        mesh.settings.get<EZSeamCornerPrefType>("z_seam_corner"),
        mesh.settings.get<coord_t>("wall_line_width_0") * 2);
    const bool spiralize = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<bool>("magic_spiralize");
    gcode_layer.addPolygonsByOptimizer(polygons, mesh_config.inset0_config, z_seam_config, mesh.settings.get<coord_t>("wall_0_wipe_dist"), spiralize);

    addMeshOpenPolyLinesToGCode(mesh, mesh_config, gcode_layer);
}

void FffGcodeWriter::addMeshOpenPolyLinesToGCode(const SliceMeshStorage& mesh, const MeshPathConfigs& mesh_config, LayerPlan& gcode_layer) const
{
    const SliceLayer* layer = &mesh.layers[gcode_layer.getLayerNr()];

    gcode_layer.addLinesByOptimizer(layer->openPolyLines, mesh_config.inset0_config, SpaceFillType::PolyLines);
}

void FffGcodeWriter::addMeshLayerToGCode(
    const SliceDataStorage& storage,
    const std::shared_ptr<SliceMeshStorage>& mesh_ptr,
    const size_t extruder_nr,
    const MeshPathConfigs& mesh_config,
    LayerPlan& gcode_layer) const
{
    const auto& mesh = *mesh_ptr;
    if (gcode_layer.getLayerNr() > mesh.layer_nr_max_filled_layer)
    {
        return;
    }

    if (mesh.settings.get<bool>("anti_overhang_mesh") || mesh.settings.get<bool>("support_mesh"))
    {
        return;
    }

    const SliceLayer& layer = mesh.layers[gcode_layer.getLayerNr()];

    if (layer.parts.empty())
    {
        return;
    }

    gcode_layer.setMesh(mesh_ptr);

    ZSeamConfig z_seam_config;
    if (mesh.isPrinted()) //"normal" meshes with walls, skin, infill, etc. get the traditional part ordering based on the z-seam settings.
    {
        z_seam_config = ZSeamConfig(
            mesh.settings.get<EZSeamType>("z_seam_type"),
            mesh.getZSeamHint(),
            mesh.settings.get<EZSeamCornerPrefType>("z_seam_corner"),
            mesh.settings.get<coord_t>("wall_line_width_0") * 2);
    }
    PathOrderOptimizer<const SliceLayerPart*> part_order_optimizer(gcode_layer.getLastPlannedPositionOrStartingPosition(), z_seam_config);
    for (const SliceLayerPart& part : layer.parts)
    {
        part_order_optimizer.addPolygon(&part);
    }
    part_order_optimizer.optimize(false);
    for (const PathOrdering<const SliceLayerPart*>& path : part_order_optimizer.paths)
    {
        addMeshPartToGCode(storage, mesh, extruder_nr, mesh_config, *path.vertices, gcode_layer);
    }

    const std::string extruder_identifier = (mesh.settings.get<size_t>("roofing_layer_count") > 0) ? "roofing_extruder_nr" : "top_bottom_extruder_nr";
    if (extruder_nr == mesh.settings.get<ExtruderTrain&>(extruder_identifier).extruder_nr)
    {
        processIroning(storage, mesh, layer, mesh_config.ironing_config, gcode_layer);
    }
    if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr)
    {
        addMeshOpenPolyLinesToGCode(mesh, mesh_config, gcode_layer);
    }
    gcode_layer.setMesh(nullptr);
}

void FffGcodeWriter::addMeshPartToGCode(
    const SliceDataStorage& storage,
    const SliceMeshStorage& mesh,
    const size_t extruder_nr,
    const MeshPathConfigs& mesh_config,
    const SliceLayerPart& part,
    LayerPlan& gcode_layer) const
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;

    bool added_something = false;

    if (mesh.settings.get<bool>("infill_before_walls"))
    {
        added_something = added_something | processInfill(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);
    }

    added_something = added_something | processInsets(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);

    if (! mesh.settings.get<bool>("infill_before_walls"))
    {
        added_something = added_something | processInfill(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);
    }

    added_something = added_something | processSkin(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);

    // After a layer part, make sure the nozzle is inside the comb boundary, so we do not retract on the perimeter.
    if (added_something
        && (! mesh_group_settings.get<bool>("magic_spiralize") || gcode_layer.getLayerNr() < static_cast<LayerIndex>(mesh.settings.get<size_t>("initial_bottom_layers"))))
    {
        coord_t innermost_wall_line_width = mesh.settings.get<coord_t>((mesh.settings.get<size_t>("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0");
        if (gcode_layer.getLayerNr() == 0)
        {
            innermost_wall_line_width *= mesh.settings.get<Ratio>("initial_layer_line_width_factor");
        }
        gcode_layer.moveInsideCombBoundary(innermost_wall_line_width, part);
    }

    gcode_layer.setIsInside(false);
}

bool FffGcodeWriter::processInfill(
    const SliceDataStorage& storage,
    LayerPlan& gcode_layer,
    const SliceMeshStorage& mesh,
    const size_t extruder_nr,
    const MeshPathConfigs& mesh_config,
    const SliceLayerPart& part) const
{
    if (extruder_nr != mesh.settings.get<ExtruderTrain&>("infill_extruder_nr").extruder_nr)
    {
        return false;
    }
    bool added_something = processMultiLayerInfill(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);
    added_something = added_something | processSingleLayerInfill(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);
    return added_something;
}

bool FffGcodeWriter::processMultiLayerInfill(
    const SliceDataStorage& storage,
    LayerPlan& gcode_layer,
    const SliceMeshStorage& mesh,
    const size_t extruder_nr,
    const MeshPathConfigs& mesh_config,
    const SliceLayerPart& part) const
{
    if (extruder_nr != mesh.settings.get<ExtruderTrain&>("infill_extruder_nr").extruder_nr)
    {
        return false;
    }
    const coord_t infill_line_distance = mesh.settings.get<coord_t>("infill_line_distance");
    if (infill_line_distance <= 0)
    {
        return false;
    }
    coord_t max_resolution = mesh.settings.get<coord_t>("meshfix_maximum_resolution");
    coord_t max_deviation = mesh.settings.get<coord_t>("meshfix_maximum_deviation");
    AngleDegrees infill_angle = 45; // Original default. This will get updated to an element from mesh->infill_angles.
    if (! mesh.infill_angles.empty())
    {
        const size_t combined_infill_layers
            = std::max(uint64_t(1), round_divide(mesh.settings.get<coord_t>("infill_sparse_thickness"), std::max(mesh.settings.get<coord_t>("layer_height"), coord_t(1))));
        infill_angle = mesh.infill_angles.at((gcode_layer.getLayerNr() / combined_infill_layers) % mesh.infill_angles.size());
    }
    const Point3 mesh_middle = mesh.bounding_box.getMiddle();
    const Point infill_origin(mesh_middle.x + mesh.settings.get<coord_t>("infill_offset_x"), mesh_middle.y + mesh.settings.get<coord_t>("infill_offset_y"));

    // Print the thicker infill lines first. (double or more layer thickness, infill combined with previous layers)
    bool added_something = false;
    for (unsigned int combine_idx = 1; combine_idx < part.infill_area_per_combine_per_density[0].size(); combine_idx++)
    {
        const coord_t infill_line_width = mesh_config.infill_config[combine_idx].getLineWidth();
        const EFillMethod infill_pattern = mesh.settings.get<EFillMethod>("infill_pattern");
        const bool zig_zaggify_infill = mesh.settings.get<bool>("zig_zaggify_infill") || infill_pattern == EFillMethod::ZIG_ZAG;
        const bool connect_polygons = mesh.settings.get<bool>("connect_infill_polygons");
        const size_t infill_multiplier = mesh.settings.get<size_t>("infill_multiplier");
        Polygons infill_polygons;
        Polygons infill_lines;
        std::vector<VariableWidthLines> infill_paths = part.infill_wall_toolpaths;
        for (size_t density_idx = part.infill_area_per_combine_per_density.size() - 1; (int)density_idx >= 0; density_idx--)
        { // combine different density infill areas (for gradual infill)
            size_t density_factor = 2 << density_idx; // == pow(2, density_idx + 1)
            coord_t infill_line_distance_here = infill_line_distance * density_factor; // the highest density infill combines with the next to create a grid with density_factor 1
            coord_t infill_shift = infill_line_distance_here / 2;
            if (density_idx == part.infill_area_per_combine_per_density.size() - 1 || infill_pattern == EFillMethod::CROSS || infill_pattern == EFillMethod::CROSS_3D)
            {
                infill_line_distance_here /= 2;
            }

            constexpr size_t wall_line_count = 0; // wall toolpaths are when gradual infill areas are determined
            const coord_t small_area_width = 0;
            constexpr coord_t infill_overlap = 0; // Overlap is handled when the wall toolpaths are generated
            constexpr bool skip_stitching = false;
            constexpr bool connected_zigzags = false;
            constexpr bool use_endpieces = true;
            constexpr bool skip_some_zags = false;
            constexpr size_t zag_skip_count = 0;
            const bool fill_gaps = density_idx == 0; // Only fill gaps for the lowest density.

            std::shared_ptr<LightningLayer> lightning_layer = nullptr;
            if (mesh.lightning_generator)
            {
                lightning_layer = std::make_shared<LightningLayer>(mesh.lightning_generator->getTreesForLayer(gcode_layer.getLayerNr()));
            }
            Infill infill_comp(
                infill_pattern,
                zig_zaggify_infill,
                connect_polygons,
                part.infill_area_per_combine_per_density[density_idx][combine_idx],
                infill_line_width,
                infill_line_distance_here,
                infill_overlap,
                infill_multiplier,
                infill_angle,
                gcode_layer.z,
                infill_shift,
                max_resolution,
                max_deviation,
                wall_line_count,
                small_area_width,
                infill_origin,
                skip_stitching,
                fill_gaps,
                connected_zigzags,
                use_endpieces,
                skip_some_zags,
                zag_skip_count,
                mesh.settings.get<coord_t>("cross_infill_pocket_size"));
            infill_comp.generate(
                infill_paths,
                infill_polygons,
                infill_lines,
                mesh.settings,
                gcode_layer.getLayerNr(),
                SectionType::INFILL,
                mesh.cross_fill_provider,
                lightning_layer,
                &mesh);
        }
        if (! infill_lines.empty() || ! infill_polygons.empty())
        {
            added_something = true;
            gcode_layer.setIsInside(true); // going to print stuff inside print object

            if (! infill_polygons.empty())
            {
                constexpr bool force_comb_retract = false;
                gcode_layer.addTravel(infill_polygons[0][0], force_comb_retract);
                gcode_layer.addPolygonsByOptimizer(infill_polygons, mesh_config.infill_config[combine_idx]);
            }

            if (! infill_lines.empty())
            {
                std::optional<Point> near_start_location;
                if (mesh.settings.get<bool>("infill_randomize_start_location"))
                {
                    srand(gcode_layer.getLayerNr());
                    near_start_location = infill_lines[rand() % infill_lines.size()][0];
                }

                const bool enable_travel_optimization = mesh.settings.get<bool>("infill_enable_travel_optimization");
                gcode_layer.addLinesByOptimizer(
                    infill_lines,
                    mesh_config.infill_config[combine_idx],
                    zig_zaggify_infill ? SpaceFillType::PolyLines : SpaceFillType::Lines,
                    enable_travel_optimization,
                    /*wipe_dist = */ 0,
                    /* flow = */ 1.0,
                    near_start_location);
            }
        }
    }
    return added_something;
}

bool FffGcodeWriter::processSingleLayerInfill(
    const SliceDataStorage& storage,
    LayerPlan& gcode_layer,
    const SliceMeshStorage& mesh,
    const size_t extruder_nr,
    const MeshPathConfigs& mesh_config,
    const SliceLayerPart& part) const
{
    if (extruder_nr != mesh.settings.get<ExtruderTrain&>("infill_extruder_nr").extruder_nr)
    {
        return false;
    }
    const auto infill_line_distance = mesh.settings.get<coord_t>("infill_line_distance");
    if (infill_line_distance == 0 || part.infill_area_per_combine_per_density[0].empty())
    {
        return false;
    }
    bool added_something = false;
    const coord_t infill_line_width = mesh_config.infill_config[0].getLineWidth();

    // Combine the 1 layer thick infill with the top/bottom skin and print that as one thing.
    Polygons infill_polygons;
    std::vector<std::vector<VariableWidthLines>> wall_tool_paths; // All wall toolpaths binned by inset_idx (inner) and by density_idx (outer)
    Polygons infill_lines;

    const auto pattern = mesh.settings.get<EFillMethod>("infill_pattern");
    const bool zig_zaggify_infill = mesh.settings.get<bool>("zig_zaggify_infill") || pattern == EFillMethod::ZIG_ZAG;
    const bool connect_polygons = mesh.settings.get<bool>("connect_infill_polygons");
    const auto infill_overlap = mesh.settings.get<coord_t>("infill_overlap_mm");
    const auto infill_multiplier = mesh.settings.get<size_t>("infill_multiplier");
    const auto wall_line_count = mesh.settings.get<size_t>("infill_wall_line_count");
    const size_t last_idx = part.infill_area_per_combine_per_density.size() - 1;
    const auto max_resolution = mesh.settings.get<coord_t>("meshfix_maximum_resolution");
    const auto max_deviation = mesh.settings.get<coord_t>("meshfix_maximum_deviation");
    AngleDegrees infill_angle = 45; // Original default. This will get updated to an element from mesh->infill_angles.
    if (! mesh.infill_angles.empty())
    {
        const size_t combined_infill_layers
            = std::max(uint64_t(1), round_divide(mesh.settings.get<coord_t>("infill_sparse_thickness"), std::max(mesh.settings.get<coord_t>("layer_height"), coord_t(1))));
        infill_angle = mesh.infill_angles.at((static_cast<size_t>(gcode_layer.getLayerNr()) / combined_infill_layers) % mesh.infill_angles.size());
    }
    const Point3 mesh_middle = mesh.bounding_box.getMiddle();
    const Point infill_origin(mesh_middle.x + mesh.settings.get<coord_t>("infill_offset_x"), mesh_middle.y + mesh.settings.get<coord_t>("infill_offset_y"));

    auto get_cut_offset = [](const bool zig_zaggify, const coord_t line_width, const size_t line_count)
    {
        if (zig_zaggify)
        {
            return -line_width / 2 - static_cast<coord_t>(line_count) * line_width - 5;
        }
        return -static_cast<coord_t>(line_count) * line_width;
    };

    Polygons sparse_in_outline = part.infill_area_per_combine_per_density[last_idx][0];

    // if infill walls are required below the boundaries of skin regions above, partition the infill along the
    // boundary edge
    Polygons infill_below_skin;
    Polygons infill_not_below_skin;
    const bool hasSkinEdgeSupport = partitionInfillBySkinAbove(infill_below_skin, infill_not_below_skin, gcode_layer, mesh, part, infill_line_width);

    const auto pocket_size = mesh.settings.get<coord_t>("cross_infill_pocket_size");
    constexpr bool skip_stitching = false;
    constexpr bool connected_zigzags = false;
    const bool use_endpieces = part.infill_area_per_combine_per_density.size() == 1; // Only use endpieces when not using gradual infill, since they will then overlap.
    constexpr bool skip_some_zags = false;
    constexpr int zag_skip_count = 0;

    for (size_t density_idx = last_idx; static_cast<int>(density_idx) >= 0; density_idx--)
    {
        // Only process dense areas when they're initialized
        if (part.infill_area_per_combine_per_density[density_idx][0].empty())
        {
            continue;
        }

        Polygons infill_lines_here;
        Polygons infill_polygons_here;

        // the highest density infill combines with the next to create a grid with density_factor 1
        int infill_line_distance_here = infill_line_distance << (density_idx + 1);
        int infill_shift = infill_line_distance_here / 2;

        /* infill shift explanation: [>]=shift ["]=line_dist

         :       |       :       |       :       |       :       |         > furthest from top
         :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |     > further from top
         : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | |   > near top
         >>"""""
         :       |       :       |       :       |       :       |         > furthest from top
         :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |     > further from top
         : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | |   > near top
         >>>>"""""""""
         :       |       :       |       :       |       :       |         > furthest from top
         :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |     > further from top
         : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | |   > near top
         >>>>>>>>"""""""""""""""""
         */

        // All of that doesn't hold for the Cross patterns; they should just always be multiplied by 2.
        if (density_idx == part.infill_area_per_combine_per_density.size() - 1 || pattern == EFillMethod::CROSS || pattern == EFillMethod::CROSS_3D)
        {
            /* the least dense infill should fill up all remaining gaps
             :       |       :       |       :       |       :       |       :  > furthest from top
             :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |   :  > further from top
             : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | | :  > near top
               .   .     .       .           .               .       .       .
               :   :     :       :           :               :       :       :
               `"""'     `"""""""'           `"""""""""""""""'       `"""""""'
                                                                         ^   new line distance for lowest density infill
                                                   ^ infill_line_distance_here for lowest density infill up till here
                             ^ middle density line dist
                 ^   highest density line dist*/

            // All of that doesn't hold for the Cross patterns; they should just always be multiplied by 2 for every density index.
            infill_line_distance_here /= 2;
        }

        Polygons in_outline = part.infill_area_per_combine_per_density[density_idx][0];

        std::shared_ptr<LightningLayer> lightning_layer;
        if (mesh.lightning_generator)
        {
            lightning_layer = std::make_shared<LightningLayer>(mesh.lightning_generator->getTreesForLayer(gcode_layer.getLayerNr()));
        }

        const bool fill_gaps = density_idx == 0; // Only fill gaps in the lowest infill density pattern.
        if (hasSkinEdgeSupport)
        {
            // infill region with skin above has to have at least one infill wall line
            const size_t min_skin_below_wall_count = wall_line_count > 0 ? wall_line_count : 1;
            const size_t skin_below_wall_count = density_idx == last_idx ? min_skin_below_wall_count : 0;
            const coord_t small_area_width = 0;
            wall_tool_paths.emplace_back(std::vector<VariableWidthLines>());
            const coord_t overlap = infill_overlap - (density_idx == last_idx ? 0 : wall_line_count * infill_line_width);
            Infill infill_comp(
                pattern,
                zig_zaggify_infill,
                connect_polygons,
                infill_below_skin,
                infill_line_width,
                infill_line_distance_here,
                overlap,
                infill_multiplier,
                infill_angle,
                gcode_layer.z,
                infill_shift,
                max_resolution,
                max_deviation,
                skin_below_wall_count,
                small_area_width,
                infill_origin,
                skip_stitching,
                fill_gaps,
                connected_zigzags,
                use_endpieces,
                skip_some_zags,
                zag_skip_count,
                pocket_size);
            infill_comp.generate(
                wall_tool_paths.back(),
                infill_polygons,
                infill_lines,
                mesh.settings,
                gcode_layer.getLayerNr(),
                SectionType::INFILL,
                mesh.cross_fill_provider,
                lightning_layer,
                &mesh);
            if (density_idx < last_idx)
            {
                const coord_t cut_offset = get_cut_offset(zig_zaggify_infill, infill_line_width, min_skin_below_wall_count);
                Polygons tool = infill_below_skin.offset(static_cast<int>(cut_offset));
                infill_lines_here = tool.intersectionPolyLines(infill_lines_here);
            }
            infill_lines.add(infill_lines_here);
            // normal processing for the infill that isn't below skin
            in_outline = infill_not_below_skin;
            if (density_idx == last_idx)
            {
                sparse_in_outline = infill_not_below_skin;
            }
        }

        const coord_t circumference = in_outline.polygonLength();
        // Originally an area of 0.4*0.4*2 (2 line width squares) was found to be a good threshold for removal.
        // However we found that this doesn't scale well with polygons with larger circumference (https://github.com/Ultimaker/Cura/issues/3992).
        // Given that the original test worked for approximately 2x2cm models, this scaling by circumference should make it work for any size.
        constexpr double minimum_small_area_factor = 0.4 * 0.4 / 40000;
        const double minimum_small_area = minimum_small_area_factor * circumference;

        // This is only for density infill, because after generating the infill might appear unnecessary infill on walls
        // especially on vertical surfaces
        in_outline.removeSmallAreas(minimum_small_area);

        constexpr size_t wall_line_count_here = 0; // Wall toolpaths were generated in generateGradualInfill for the sparsest density, denser parts don't have walls by default
        const coord_t small_area_width = 0;
        constexpr coord_t overlap = 0; // overlap is already applied for the sparsest density in the generateGradualInfill

        wall_tool_paths.emplace_back();
        Infill infill_comp(
            pattern,
            zig_zaggify_infill,
            connect_polygons,
            in_outline,
            infill_line_width,
            infill_line_distance_here,
            overlap,
            infill_multiplier,
            infill_angle,
            gcode_layer.z,
            infill_shift,
            max_resolution,
            max_deviation,
            wall_line_count_here,
            small_area_width,
            infill_origin,
            skip_stitching,
            fill_gaps,
            connected_zigzags,
            use_endpieces,
            skip_some_zags,
            zag_skip_count,
            pocket_size);
        infill_comp.generate(
            wall_tool_paths.back(),
            infill_polygons,
            infill_lines,
            mesh.settings,
            gcode_layer.getLayerNr(),
            SectionType::INFILL,
            mesh.cross_fill_provider,
            lightning_layer,
            &mesh);
        if (density_idx < last_idx)
        {
            const coord_t cut_offset = get_cut_offset(zig_zaggify_infill, infill_line_width, wall_line_count);
            Polygons tool = sparse_in_outline.offset(static_cast<int>(cut_offset));
            infill_lines_here = tool.intersectionPolyLines(infill_lines_here);
        }
        infill_lines.add(infill_lines_here);
        infill_polygons.add(infill_polygons_here);
    }

    wall_tool_paths.emplace_back(part.infill_wall_toolpaths); // The extra infill walls were generated separately. Add these too.
    const bool walls_generated = std::any_of(
        wall_tool_paths.cbegin(),
        wall_tool_paths.cend(),
        [](const std::vector<VariableWidthLines>& tp)
        {
            return ! (
                tp.empty()
                || std::all_of(
                    tp.begin(),
                    tp.end(),
                    [](const VariableWidthLines& vwl)
                    {
                        return vwl.empty();
                    }));
        });
    if (! infill_lines.empty() || ! infill_polygons.empty() || walls_generated)
    {
        added_something = true;
        gcode_layer.setIsInside(true); // going to print stuff inside print object
        std::optional<Point> near_start_location;
        if (mesh.settings.get<bool>("infill_randomize_start_location"))
        {
            srand(gcode_layer.getLayerNr());
            if (! infill_lines.empty())
            {
                near_start_location = infill_lines[rand() % infill_lines.size()][0];
            }
            else if (! infill_polygons.empty())
            {
                PolygonRef start_poly = infill_polygons[rand() % infill_polygons.size()];
                near_start_location = start_poly[rand() % start_poly.size()];
            }
            else // So walls_generated must be true.
            {
                std::vector<VariableWidthLines>* start_paths = &wall_tool_paths[rand() % wall_tool_paths.size()];
                while (start_paths->empty() || (*start_paths)[0].empty()) // We know for sure (because walls_generated) that one of them is not empty. So randomise until we hit it.
                                                                          // Should almost always be very quick.
                {
                    start_paths = &wall_tool_paths[rand() % wall_tool_paths.size()];
                }
                near_start_location = (*start_paths)[0][0].junctions[0].p;
            }
        }
        if (walls_generated)
        {
            for (const std::vector<VariableWidthLines>& tool_paths : wall_tool_paths)
            {
                constexpr bool retract_before_outer_wall = false;
                constexpr coord_t wipe_dist = 0;
                const ZSeamConfig z_seam_config(
                    mesh.settings.get<EZSeamType>("z_seam_type"),
                    mesh.getZSeamHint(),
                    mesh.settings.get<EZSeamCornerPrefType>("z_seam_corner"),
                    mesh_config.infill_config[0].getLineWidth() * 2);
                InsetOrderOptimizer wall_orderer(
                    *this,
                    storage,
                    gcode_layer,
                    mesh.settings,
                    extruder_nr,
                    mesh_config.infill_config[0],
                    mesh_config.infill_config[0],
                    mesh_config.infill_config[0],
                    mesh_config.infill_config[0],
                    retract_before_outer_wall,
                    wipe_dist,
                    wipe_dist,
                    extruder_nr,
                    extruder_nr,
                    z_seam_config,
                    tool_paths);
                added_something |= wall_orderer.addToLayer();
            }
        }
        if (! infill_polygons.empty())
        {
            constexpr bool force_comb_retract = false;
            // start the infill polygons at the nearest vertex to the current location
            gcode_layer.addTravel(PolygonUtils::findNearestVert(gcode_layer.getLastPlannedPositionOrStartingPosition(), infill_polygons).p(), force_comb_retract);
            gcode_layer.addPolygonsByOptimizer(infill_polygons, mesh_config.infill_config[0], ZSeamConfig(), 0, false, 1.0_r, false, false, near_start_location);
        }
        const bool enable_travel_optimization = mesh.settings.get<bool>("infill_enable_travel_optimization");
        if (pattern == EFillMethod::GRID || pattern == EFillMethod::LINES || pattern == EFillMethod::TRIANGLES || pattern == EFillMethod::CUBIC
            || pattern == EFillMethod::TETRAHEDRAL || pattern == EFillMethod::QUARTER_CUBIC || pattern == EFillMethod::CUBICSUBDIV || pattern == EFillMethod::LIGHTNING)
        {
            gcode_layer.addLinesByOptimizer(
                infill_lines,
                mesh_config.infill_config[0],
                SpaceFillType::Lines,
                enable_travel_optimization,
                mesh.settings.get<coord_t>("infill_wipe_dist"),
                /*float_ratio = */ 1.0,
                near_start_location);
        }
        else
        {
            gcode_layer.addLinesByOptimizer(
                infill_lines,
                mesh_config.infill_config[0],
                (pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines,
                enable_travel_optimization,
                /* wipe_dist = */ 0,
                /*float_ratio = */ 1.0,
                near_start_location);
        }
    }
    return added_something;
}

bool FffGcodeWriter::partitionInfillBySkinAbove(
    Polygons& infill_below_skin,
    Polygons& infill_not_below_skin,
    const LayerPlan& gcode_layer,
    const SliceMeshStorage& mesh,
    const SliceLayerPart& part,
    coord_t infill_line_width)
{
    constexpr coord_t tiny_infill_offset = 20;
    const auto skin_edge_support_layers = mesh.settings.get<size_t>("skin_edge_support_layers");
    Polygons skin_above_combined; // skin regions on the layers above combined with small gaps between

    // working from the highest layer downwards, combine the regions of skin on all the layers
    // but don't let the regions merge together
    // otherwise "terraced" skin regions on separate layers will look like a single region of unbroken skin
    for (size_t i = skin_edge_support_layers; i > 0; --i)
    {
        const size_t skin_layer_nr = gcode_layer.getLayerNr() + i;
        if (skin_layer_nr < mesh.layers.size())
        {
            for (const SliceLayerPart& part_i : mesh.layers[skin_layer_nr].parts)
            {
                for (const SkinPart& skin_part : part_i.skin_parts)
                {
                    // Limit considered areas to the ones that should have infill underneath at the current layer.
                    const Polygons relevant_outline = skin_part.outline.intersection(part.getOwnInfillArea());

                    if (! skin_above_combined.empty())
                    {
                        // does this skin part overlap with any of the skin parts on the layers above?
                        const Polygons overlap = skin_above_combined.intersection(relevant_outline);
                        if (! overlap.empty())
                        {
                            // yes, it overlaps, need to leave a gap between this skin part and the others
                            if (i > 1) // this layer is the 2nd or higher layer above the layer whose infill we're printing
                            {
                                // looking from the side, if the combined regions so far look like this...
                                //
                                //     ----------------------------------
                                //
                                // and the new skin part looks like this...
                                //
                                //             -------------------------------------
                                //
                                // the result should be like this...
                                //
                                //     ------- -------------------------- ----------

                                // expand the overlap region slightly to make a small gap
                                const Polygons overlap_expanded = overlap.offset(tiny_infill_offset);
                                // subtract the expanded overlap region from the regions accumulated from higher layers
                                skin_above_combined = skin_above_combined.difference(overlap_expanded);
                                // subtract the expanded overlap region from this skin part and add the remainder to the overlap region
                                skin_above_combined.add(relevant_outline.difference(overlap_expanded));
                                // and add the overlap area as well
                                skin_above_combined.add(overlap);
                            }
                            else // this layer is the 1st layer above the layer whose infill we're printing
                            {
                                // add this layer's skin region without subtracting the overlap but still make a gap between this skin region and what has been accumulated so far
                                // we do this so that these skin region edges will definitely have infill walls below them

                                // looking from the side, if the combined regions so far look like this...
                                //
                                //     ----------------------------------
                                //
                                // and the new skin part looks like this...
                                //
                                //             -------------------------------------
                                //
                                // the result should be like this...
                                //
                                //     ------- -------------------------------------

                                skin_above_combined = skin_above_combined.difference(relevant_outline.offset(tiny_infill_offset));
                                skin_above_combined.add(relevant_outline);
                            }
                        }
                        else // no overlap
                        {
                            skin_above_combined.add(relevant_outline);
                        }
                    }
                    else // this is the first skin region we have looked at
                    {
                        skin_above_combined.add(relevant_outline);
                    }
                }
            }
        }

        // the shrink/expand here is to remove regions of infill below skin that are narrower than the width of the infill walls otherwise the infill walls could merge and form a
        // bump
        infill_below_skin = skin_above_combined.intersection(part.infill_area_per_combine_per_density.back().front()).offset(-infill_line_width).offset(infill_line_width);

        constexpr bool remove_small_holes_from_infill_below_skin = true;
        constexpr double min_area_multiplier = 25;
        const double min_area = INT2MM(infill_line_width) * INT2MM(infill_line_width) * min_area_multiplier;
        infill_below_skin.removeSmallAreas(min_area, remove_small_holes_from_infill_below_skin);

        // there is infill below skin, is there also infill that isn't below skin?
        infill_not_below_skin = part.infill_area_per_combine_per_density.back().front().difference(infill_below_skin);
        infill_not_below_skin.removeSmallAreas(min_area);
    }

    // need to take skin/infill overlap that was added in SkinInfillAreaComputation::generateInfill() into account
    const coord_t infill_skin_overlap = mesh.settings.get<coord_t>((part.wall_toolpaths.size() > 1) ? "wall_line_width_x" : "wall_line_width_0") / 2;
    const Polygons infill_below_skin_overlap = infill_below_skin.offset(-(infill_skin_overlap + tiny_infill_offset));

    return ! infill_below_skin_overlap.empty() && ! infill_not_below_skin.empty();
}

void FffGcodeWriter::processSpiralizedWall(
    const SliceDataStorage& storage,
    LayerPlan& gcode_layer,
    const MeshPathConfigs& mesh_config,
    const SliceLayerPart& part,
    const SliceMeshStorage& mesh) const
{
    if (part.spiral_wall.empty())
    {
        // wall doesn't have usable outline
        return;
    }
    const ClipperLib::Path* last_wall_outline = &*part.spiral_wall[0]; // default to current wall outline
    int last_seam_vertex_idx = -1; // last layer seam vertex index
    int layer_nr = gcode_layer.getLayerNr();
    if (layer_nr > 0)
    {
        if (storage.spiralize_wall_outlines[layer_nr - 1] != nullptr)
        {
            // use the wall outline from the previous layer
            last_wall_outline = &*(*storage.spiralize_wall_outlines[layer_nr - 1])[0];
            // and the seam vertex index pre-computed for that layer
            last_seam_vertex_idx = storage.spiralize_seam_vertex_indices[layer_nr - 1];
        }
    }
    const bool is_bottom_layer = (layer_nr == mesh.settings.get<LayerIndex>("initial_bottom_layers"));
    const bool is_top_layer = ((size_t)layer_nr == (storage.spiralize_wall_outlines.size() - 1) || storage.spiralize_wall_outlines[layer_nr + 1] == nullptr);
    const int seam_vertex_idx = storage.spiralize_seam_vertex_indices[layer_nr]; // use pre-computed seam vertex index for current layer
    // output a wall slice that is interpolated between the last and current walls
    for (const ConstPolygonRef& wall_outline : part.spiral_wall)
    {
        gcode_layer
            .spiralizeWallSlice(mesh_config.inset0_config, wall_outline, ConstPolygonRef(*last_wall_outline), seam_vertex_idx, last_seam_vertex_idx, is_top_layer, is_bottom_layer);
    }
}

bool FffGcodeWriter::processInsets(
    const SliceDataStorage& storage,
    LayerPlan& gcode_layer,
    const SliceMeshStorage& mesh,
    const size_t extruder_nr,
    const MeshPathConfigs& mesh_config,
    const SliceLayerPart& part) const
{
    bool added_something = false;
    if (extruder_nr != mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr && extruder_nr != mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr)
    {
        return added_something;
    }
    if (mesh.settings.get<size_t>("wall_line_count") <= 0)
    {
        return added_something;
    }

    bool spiralize = false;
    if (Application::getInstance().current_slice->scene.current_mesh_group->settings.get<bool>("magic_spiralize"))
    {
        const size_t initial_bottom_layers = mesh.settings.get<size_t>("initial_bottom_layers");
        const int layer_nr = gcode_layer.getLayerNr();
        if ((layer_nr < static_cast<LayerIndex>(initial_bottom_layers)
             && part.wall_toolpaths.empty()) // The bottom layers in spiralize mode are generated using the variable width paths
            || (layer_nr >= static_cast<LayerIndex>(initial_bottom_layers) && part.spiral_wall.empty())) // The rest of the layers in spiralize mode are using the spiral wall
        {
            // nothing to do
            return false;
        }
        if (gcode_layer.getLayerNr() >= static_cast<LayerIndex>(initial_bottom_layers))
        {
            spiralize = true;
        }
        if (spiralize && gcode_layer.getLayerNr() == static_cast<LayerIndex>(initial_bottom_layers)
            && extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr)
        { // on the last normal layer first make the outer wall normally and then start a second outer wall from the same hight, but gradually moving upward
            added_something = true;
            gcode_layer.setIsInside(true); // going to print stuff inside print object
            // start this first wall at the same vertex the spiral starts
            const ConstPolygonRef spiral_inset = part.spiral_wall[0];
            const size_t spiral_start_vertex = storage.spiralize_seam_vertex_indices[initial_bottom_layers];
            if (spiral_start_vertex < spiral_inset.size())
            {
                gcode_layer.addTravel(spiral_inset[spiral_start_vertex]);
            }
            int wall_0_wipe_dist(0);
            gcode_layer.addPolygonsByOptimizer(part.spiral_wall, mesh_config.inset0_config, ZSeamConfig(), wall_0_wipe_dist);
        }
    }
    // for non-spiralized layers, determine the shape of the unsupported areas below this part
    if (! spiralize && gcode_layer.getLayerNr() > 0)
    {
        // accumulate the outlines of all of the parts that are on the layer below

        Polygons outlines_below;
        AABB boundaryBox(part.outline);
        for (const std::shared_ptr<SliceMeshStorage>& mesh_ptr : storage.meshes)
        {
            const auto& m = *mesh_ptr;
            if (m.isPrinted())
            {
                for (const SliceLayerPart& prevLayerPart : m.layers[gcode_layer.getLayerNr() - 1].parts)
                {
                    if (boundaryBox.hit(prevLayerPart.boundaryBox))
                    {
                        outlines_below.add(prevLayerPart.outline);
                    }
                }
            }
        }

        const coord_t layer_height = mesh_config.inset0_config.getLayerThickness();

        // if support is enabled, add the support outlines also so we don't generate bridges over support

        const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
        if (mesh_group_settings.get<bool>("support_enable"))
        {
            const coord_t z_distance_top = mesh.settings.get<coord_t>("support_top_distance");
            const size_t z_distance_top_layers = (z_distance_top / layer_height) + 1;
            const int support_layer_nr = gcode_layer.getLayerNr() - z_distance_top_layers;

            if (support_layer_nr > 0)
            {
                const SupportLayer& support_layer = storage.support.supportLayers[support_layer_nr];

                if (! support_layer.support_roof.empty())
                {
                    AABB support_roof_bb(support_layer.support_roof);
                    if (boundaryBox.hit(support_roof_bb))
                    {
                        outlines_below.add(support_layer.support_roof);
                    }
                }
                else
                {
                    for (const SupportInfillPart& support_part : support_layer.support_infill_parts)
                    {
                        AABB support_part_bb(support_part.getInfillArea());
                        if (boundaryBox.hit(support_part_bb))
                        {
                            outlines_below.add(support_part.getInfillArea());
                        }
                    }
                }
            }
        }

        const int half_outer_wall_width = mesh_config.inset0_config.getLineWidth() / 2;

        // remove those parts of the layer below that are narrower than a wall line width as they will not be printed

        outlines_below = outlines_below.offset(-half_outer_wall_width).offset(half_outer_wall_width);

        if (mesh.settings.get<bool>("bridge_settings_enabled"))
        {
            // max_air_gap is the max allowed width of the unsupported region below the wall line
            // if the unsupported region is wider than max_air_gap, the wall line will be printed using bridge settings

            const coord_t max_air_gap = half_outer_wall_width;

            // subtract the outlines of the parts below this part to give the shapes of the unsupported regions and then
            // shrink those shapes so that any that are narrower than two times max_air_gap will be removed

            Polygons compressed_air(part.outline.difference(outlines_below).offset(-max_air_gap));

            // now expand the air regions by the same amount as they were shrunk plus half the outer wall line width
            // which is required because when the walls are being generated, the vertices do not fall on the part's outline
            // but, instead, are 1/2 a line width inset from the outline

            gcode_layer.setBridgeWallMask(compressed_air.offset(max_air_gap + half_outer_wall_width));
        }
        else
        {
            // clear to disable use of bridging settings
            gcode_layer.setBridgeWallMask(Polygons());
        }

        const AngleDegrees overhang_angle = mesh.settings.get<AngleDegrees>("wall_overhang_angle");
        if (overhang_angle >= 90)
        {
            // clear to disable overhang detection
            gcode_layer.setOverhangMask(Polygons());
        }
        else
        {
            // the overhang mask is set to the area of the current part's outline minus the region that is considered to be supported
            // the supported region is made up of those areas that really are supported by either model or support on the layer below
            // expanded to take into account the overhang angle, the greater the overhang angle, the larger the supported area is
            // considered to be
            const coord_t overhang_width = layer_height * std::tan(overhang_angle / (180 / M_PI));
            Polygons overhang_region = part.outline.offset(-half_outer_wall_width).difference(outlines_below.offset(10 + overhang_width - half_outer_wall_width)).offset(10);
            gcode_layer.setOverhangMask(overhang_region);
        }
    }
    else
    {
        // clear to disable use of bridging settings
        gcode_layer.setBridgeWallMask(Polygons());
        // clear to disable overhang detection
        gcode_layer.setOverhangMask(Polygons());
    }

    if (spiralize && extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr && ! part.spiral_wall.empty())
    {
        added_something = true;
        gcode_layer.setIsInside(true); // going to print stuff inside print object

        // Only spiralize the first part in the mesh, any other parts will be printed using the normal, non-spiralize codepath.
        // This sounds weird but actually does the right thing when you have a model that has multiple parts at the bottom that merge into
        // one part higher up. Once all the parts have merged, layers above that level will be spiralized
        if (&mesh.layers[gcode_layer.getLayerNr()].parts[0] == &part)
        {
            processSpiralizedWall(storage, gcode_layer, mesh_config, part, mesh);
        }
        else
        {
            // Print the spiral walls of other parts as single walls without Z gradient.
            gcode_layer.addWalls(part.spiral_wall, mesh.settings, mesh_config.inset0_config, mesh_config.inset0_config);
        }
    }
    else
    {
        // for layers that (partially) do not have any layers above we apply the roofing configuration
        auto use_roofing_config = [&part, &mesh, &gcode_layer]()
        {
            const auto getOutlineOnLayer = [mesh](const SliceLayerPart& part_here, const LayerIndex layer2_nr) -> Polygons
            {
                Polygons result;
                if (layer2_nr >= static_cast<int>(mesh.layers.size()))
                {
                    return result;
                }
                const SliceLayer& layer2 = mesh.layers[layer2_nr];
                for (const SliceLayerPart& part2 : layer2.parts)
                {
                    if (part_here.boundaryBox.hit(part2.boundaryBox))
                    {
                        result.add(part2.outline);
                    }
                }
                return result;
            };

            const auto filled_area_above = [&getOutlineOnLayer, &part, &mesh, &gcode_layer]() -> Polygons
            {
                const size_t roofing_layer_count = std::min(mesh.settings.get<size_t>("roofing_layer_count"), mesh.settings.get<size_t>("top_layers"));
                const bool no_small_gaps_heuristic = mesh.settings.get<bool>("skin_no_small_gaps_heuristic");
                const int layer_nr = gcode_layer.getLayerNr();
                auto filled_area_above = getOutlineOnLayer(part, layer_nr + roofing_layer_count);
                if (! no_small_gaps_heuristic)
                {
                    for (int layer_nr_above = layer_nr + 1; layer_nr_above < layer_nr + roofing_layer_count; layer_nr_above++)
                    {
                        Polygons outlines_above = getOutlineOnLayer(part, layer_nr_above);
                        filled_area_above = filled_area_above.intersection(outlines_above);
                    }
                }
                if (layer_nr > 0)
                {
                    // if the skin has air below it then cutting it into regions could cause a region
                    // to be wholely or partly above air and it may not be printable so restrict
                    // the regions that have air above (the visible regions) to not include any area that
                    // has air below (fixes https://github.com/Ultimaker/Cura/issues/2656)

                    // set air_below to the skin area for the current layer that has air below it
                    Polygons air_below = getOutlineOnLayer(part, layer_nr).difference(getOutlineOnLayer(part, layer_nr - 1));

                    if (! air_below.empty())
                    {
                        // add the polygons that have air below to the no air above polygons
                        filled_area_above = filled_area_above.unionPolygons(air_below);
                    }
                }

                return filled_area_above;
            }();

            if (filled_area_above.empty())
            {
                return true;
            }

            const auto point_view = ranges::views::transform(
                [](auto extrusion_junction)
                {
                    return extrusion_junction.p;
                });

            for (const auto& path : part.wall_toolpaths)
            {
                for (const auto& wall : path)
                {
                    for (const auto& p : wall | point_view)
                    {
                        if (! filled_area_above.inside(p))
                        {
                            return true;
                        }
                    }

                    for (const auto& window : wall | point_view | ranges::views::sliding(2))
                    {
                        auto p0 = window[0];
                        auto p1 = window[1];
                        if (PolygonUtils::polygonCollidesWithLineSegment(filled_area_above, p0, p1))
                        {
                            return true;
                        }
                    }
                }
            }
            return false;
        }();

        const GCodePathConfig& inset0_config = use_roofing_config ? mesh_config.inset0_roofing_config : mesh_config.inset0_config;
        const GCodePathConfig& insetX_config = use_roofing_config ? mesh_config.insetX_roofing_config : mesh_config.insetX_config;

        // Main case: Optimize the insets with the InsetOrderOptimizer.
        const coord_t wall_x_wipe_dist = 0;
        const ZSeamConfig z_seam_config(
            mesh.settings.get<EZSeamType>("z_seam_type"),
            mesh.getZSeamHint(),
            mesh.settings.get<EZSeamCornerPrefType>("z_seam_corner"),
            mesh.settings.get<coord_t>("wall_line_width_0") * 2);
        InsetOrderOptimizer wall_orderer(
            *this,
            storage,
            gcode_layer,
            mesh.settings,
            extruder_nr,
            inset0_config,
            insetX_config,
            mesh_config.bridge_inset0_config,
            mesh_config.bridge_insetX_config,
            mesh.settings.get<bool>("travel_retract_before_outer_wall"),
            mesh.settings.get<coord_t>("wall_0_wipe_dist"),
            wall_x_wipe_dist,
            mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr,
            mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr,
            z_seam_config,
            part.wall_toolpaths);
        added_something |= wall_orderer.addToLayer();
    }
    return added_something;
}

std::optional<Point> FffGcodeWriter::getSeamAvoidingLocation(const Polygons& filling_part, int filling_angle, Point last_position) const
{
    if (filling_part.empty())
    {
        return std::optional<Point>();
    }
    // start with the BB of the outline
    AABB skin_part_bb(filling_part);
    PointMatrix rot((double)((-filling_angle + 90) % 360)); // create a matrix to rotate a vector so that it is normal to the skin angle
    const Point bb_middle = skin_part_bb.getMiddle();
    // create a vector from the middle of the BB whose length is such that it can be rotated
    // around the middle of the BB and the end will always be a long way outside of the part's outline
    // and rotate the vector so that it is normal to the skin angle
    const Point vec = rot.apply(Point(0, vSize(skin_part_bb.max - bb_middle) * 100));
    // find the vertex in the outline that is closest to the end of the rotated vector
    const PolygonsPointIndex pa = PolygonUtils::findNearestVert(bb_middle + vec, filling_part);
    // and find another outline vertex, this time using the vector + 180 deg
    const PolygonsPointIndex pb = PolygonUtils::findNearestVert(bb_middle - vec, filling_part);
    if (! pa.initialized() || ! pb.initialized())
    {
        return std::optional<Point>();
    }
    // now go to whichever of those vertices that is closest to where we are now
    if (vSize2(pa.p() - last_position) < vSize2(pb.p() - last_position))
    {
        return std::optional<Point>(std::in_place, pa.p());
    }
    else
    {
        return std::optional<Point>(std::in_place, pb.p());
    }
}

bool FffGcodeWriter::processSkin(
    const SliceDataStorage& storage,
    LayerPlan& gcode_layer,
    const SliceMeshStorage& mesh,
    const size_t extruder_nr,
    const MeshPathConfigs& mesh_config,
    const SliceLayerPart& part) const
{
    const size_t top_bottom_extruder_nr = mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr;
    const size_t roofing_extruder_nr = mesh.settings.get<ExtruderTrain&>("roofing_extruder_nr").extruder_nr;
    const size_t wall_0_extruder_nr = mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr;
    const size_t roofing_layer_count = std::min(mesh.settings.get<size_t>("roofing_layer_count"), mesh.settings.get<size_t>("top_layers"));
    if (extruder_nr != top_bottom_extruder_nr && extruder_nr != wall_0_extruder_nr && (extruder_nr != roofing_extruder_nr || roofing_layer_count <= 0))
    {
        return false;
    }
    bool added_something = false;

    PathOrderOptimizer<const SkinPart*> part_order_optimizer(gcode_layer.getLastPlannedPositionOrStartingPosition());
    for (const SkinPart& skin_part : part.skin_parts)
    {
        part_order_optimizer.addPolygon(&skin_part);
    }
    part_order_optimizer.optimize();

    for (const PathOrdering<const SkinPart*>& path : part_order_optimizer.paths)
    {
        const SkinPart& skin_part = *path.vertices;

        added_something = added_something | processSkinPart(storage, gcode_layer, mesh, extruder_nr, mesh_config, skin_part);
    }

    return added_something;
}

bool FffGcodeWriter::processSkinPart(
    const SliceDataStorage& storage,
    LayerPlan& gcode_layer,
    const SliceMeshStorage& mesh,
    const size_t extruder_nr,
    const MeshPathConfigs& mesh_config,
    const SkinPart& skin_part) const
{
    bool added_something = false;

    gcode_layer.mode_skip_agressive_merge = true;

    processRoofing(storage, gcode_layer, mesh, extruder_nr, mesh_config, skin_part, added_something);
    processTopBottom(storage, gcode_layer, mesh, extruder_nr, mesh_config, skin_part, added_something);

    gcode_layer.mode_skip_agressive_merge = false;
    return added_something;
}

void FffGcodeWriter::processRoofing(
    const SliceDataStorage& storage,
    LayerPlan& gcode_layer,
    const SliceMeshStorage& mesh,
    const size_t extruder_nr,
    const MeshPathConfigs& mesh_config,
    const SkinPart& skin_part,
    bool& added_something) const
{
    const size_t roofing_extruder_nr = mesh.settings.get<ExtruderTrain&>("roofing_extruder_nr").extruder_nr;
    if (extruder_nr != roofing_extruder_nr)
    {
        return;
    }

    const EFillMethod pattern = mesh.settings.get<EFillMethod>("roofing_pattern");
    AngleDegrees roofing_angle = 45;
    if (mesh.roofing_angles.size() > 0)
    {
        roofing_angle = mesh.roofing_angles.at(gcode_layer.getLayerNr() % mesh.roofing_angles.size());
    }

    const Ratio skin_density = 1.0;
    const coord_t skin_overlap = 0; // skinfill already expanded over the roofing areas; don't overlap with perimeters
    const bool monotonic = mesh.settings.get<bool>("roofing_monotonic");
    processSkinPrintFeature(
        storage,
        gcode_layer,
        mesh,
        mesh_config,
        extruder_nr,
        skin_part.roofing_fill,
        mesh_config.roofing_config,
        pattern,
        roofing_angle,
        skin_overlap,
        skin_density,
        monotonic,
        added_something);
}

void FffGcodeWriter::processTopBottom(
    const SliceDataStorage& storage,
    LayerPlan& gcode_layer,
    const SliceMeshStorage& mesh,
    const size_t extruder_nr,
    const MeshPathConfigs& mesh_config,
    const SkinPart& skin_part,
    bool& added_something) const
{
    if (skin_part.skin_fill.empty())
    {
        return; // bridgeAngle requires a non-empty skin_fill.
    }
    const size_t top_bottom_extruder_nr = mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr;
    if (extruder_nr != top_bottom_extruder_nr)
    {
        return;
    }
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;

    const size_t layer_nr = gcode_layer.getLayerNr();

    EFillMethod pattern = (layer_nr == 0) ? mesh.settings.get<EFillMethod>("top_bottom_pattern_0") : mesh.settings.get<EFillMethod>("top_bottom_pattern");

    AngleDegrees skin_angle = 45;
    if (mesh.skin_angles.size() > 0)
    {
        skin_angle = mesh.skin_angles.at(layer_nr % mesh.skin_angles.size());
    }

    // generate skin_polygons and skin_lines
    const GCodePathConfig* skin_config = &mesh_config.skin_config;
    Ratio skin_density = 1.0;
    const coord_t skin_overlap = 0; // Skin overlap offset is applied in skin.cpp more overlap might be beneficial for curved bridges, but makes it worse in general.
    const bool bridge_settings_enabled = mesh.settings.get<bool>("bridge_settings_enabled");
    const bool bridge_enable_more_layers = bridge_settings_enabled && mesh.settings.get<bool>("bridge_enable_more_layers");
    const Ratio support_threshold = bridge_settings_enabled ? mesh.settings.get<Ratio>("bridge_skin_support_threshold") : 0.0_r;
    const size_t bottom_layers = mesh.settings.get<size_t>("bottom_layers");

    // if support is enabled, consider the support outlines so we don't generate bridges over support

    int support_layer_nr = -1;
    const SupportLayer* support_layer = nullptr;

    if (mesh_group_settings.get<bool>("support_enable"))
    {
        const coord_t layer_height = mesh_config.inset0_config.getLayerThickness();
        const coord_t z_distance_top = mesh.settings.get<coord_t>("support_top_distance");
        const size_t z_distance_top_layers = (z_distance_top / layer_height) + 1;
        support_layer_nr = layer_nr - z_distance_top_layers;
    }

    // helper function that detects skin regions that have no support and modifies their print settings (config, line angle, density, etc.)

    auto handle_bridge_skin = [&](const int bridge_layer, const GCodePathConfig* config, const float density) // bridge_layer = 1, 2 or 3
    {
        if (support_layer_nr >= (bridge_layer - 1))
        {
            support_layer = &storage.support.supportLayers[support_layer_nr - (bridge_layer - 1)];
        }

        Polygons supported_skin_part_regions;

        const int angle = bridgeAngle(mesh.settings, skin_part.skin_fill, storage, layer_nr, bridge_layer, support_layer, supported_skin_part_regions);

        if (angle > -1 || (support_threshold > 0 && (supported_skin_part_regions.area() / (skin_part.skin_fill.area() + 1) < support_threshold)))
        {
            if (angle > -1)
            {
                switch (bridge_layer)
                {
                default:
                case 1:
                    skin_angle = angle;
                    break;

                case 2:
                    if (bottom_layers > 2)
                    {
                        // orientate second bridge skin at +45 deg to first
                        skin_angle = angle + 45;
                    }
                    else
                    {
                        // orientate second bridge skin at 90 deg to first
                        skin_angle = angle + 90;
                    }
                    break;

                case 3:
                    // orientate third bridge skin at 135 (same result as -45) deg to first
                    skin_angle = angle + 135;
                    break;
                }
            }
            pattern = EFillMethod::LINES; // force lines pattern when bridging
            if (bridge_settings_enabled)
            {
                skin_config = config;
                skin_density = density;
            }
            return true;
        }

        return false;
    };

    bool is_bridge_skin = false;
    if (layer_nr > 0)
    {
        is_bridge_skin = handle_bridge_skin(1, &mesh_config.bridge_skin_config, mesh.settings.get<Ratio>("bridge_skin_density"));
    }
    if (bridge_enable_more_layers && ! is_bridge_skin && layer_nr > 1 && bottom_layers > 1)
    {
        is_bridge_skin = handle_bridge_skin(2, &mesh_config.bridge_skin_config2, mesh.settings.get<Ratio>("bridge_skin_density_2"));

        if (! is_bridge_skin && layer_nr > 2 && bottom_layers > 2)
        {
            is_bridge_skin = handle_bridge_skin(3, &mesh_config.bridge_skin_config3, mesh.settings.get<Ratio>("bridge_skin_density_3"));
        }
    }

    double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT;

    if (layer_nr > 0 && skin_config == &mesh_config.skin_config && support_layer_nr >= 0 && mesh.settings.get<bool>("support_fan_enable"))
    {
        // skin isn't a bridge but is it above support and we need to modify the fan speed?

        AABB skin_bb(skin_part.skin_fill);

        support_layer = &storage.support.supportLayers[support_layer_nr];

        bool supported = false;

        if (! support_layer->support_roof.empty())
        {
            AABB support_roof_bb(support_layer->support_roof);
            if (skin_bb.hit(support_roof_bb))
            {
                supported = ! skin_part.skin_fill.intersection(support_layer->support_roof).empty();
            }
        }
        else
        {
            for (auto support_part : support_layer->support_infill_parts)
            {
                AABB support_part_bb(support_part.getInfillArea());
                if (skin_bb.hit(support_part_bb))
                {
                    supported = ! skin_part.skin_fill.intersection(support_part.getInfillArea()).empty();

                    if (supported)
                    {
                        break;
                    }
                }
            }
        }

        if (supported)
        {
            fan_speed = mesh.settings.get<Ratio>("support_supported_skin_fan_speed") * 100.0;
        }
    }
    const bool monotonic = mesh.settings.get<bool>("skin_monotonic");
    processSkinPrintFeature(
        storage,
        gcode_layer,
        mesh,
        mesh_config,
        extruder_nr,
        skin_part.skin_fill,
        *skin_config,
        pattern,
        skin_angle,
        skin_overlap,
        skin_density,
        monotonic,
        added_something,
        fan_speed);
}

void FffGcodeWriter::processSkinPrintFeature(
    const SliceDataStorage& storage,
    LayerPlan& gcode_layer,
    const SliceMeshStorage& mesh,
    const MeshPathConfigs& mesh_config,
    const size_t extruder_nr,
    const Polygons& area,
    const GCodePathConfig& config,
    EFillMethod pattern,
    const AngleDegrees skin_angle,
    const coord_t skin_overlap,
    const Ratio skin_density,
    const bool monotonic,
    bool& added_something,
    double fan_speed) const
{
    Polygons skin_polygons;
    Polygons skin_lines;
    std::vector<VariableWidthLines> skin_paths;

    constexpr int infill_multiplier = 1;
    constexpr int extra_infill_shift = 0;
    const size_t wall_line_count = mesh.settings.get<size_t>("skin_outline_count");
    const coord_t small_area_width = mesh.settings.get<coord_t>("small_skin_width");
    const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
    const bool connect_polygons = mesh.settings.get<bool>("connect_skin_polygons");
    coord_t max_resolution = mesh.settings.get<coord_t>("meshfix_maximum_resolution");
    coord_t max_deviation = mesh.settings.get<coord_t>("meshfix_maximum_deviation");
    const Point infill_origin;
    const bool skip_line_stitching = monotonic;
    constexpr bool fill_gaps = true;
    constexpr bool connected_zigzags = false;
    constexpr bool use_endpieces = true;
    constexpr bool skip_some_zags = false;
    constexpr int zag_skip_count = 0;
    constexpr coord_t pocket_size = 0;
    const bool small_areas_on_surface = mesh.settings.get<bool>("small_skin_on_surface");
    const auto& current_layer = mesh.layers[gcode_layer.getLayerNr()];
    const auto& exposed_to_air = current_layer.top_surface.areas.unionPolygons(current_layer.bottom_surface);

    Infill infill_comp(
        pattern,
        zig_zaggify_infill,
        connect_polygons,
        area,
        config.getLineWidth(),
        config.getLineWidth() / skin_density,
        skin_overlap,
        infill_multiplier,
        skin_angle,
        gcode_layer.z,
        extra_infill_shift,
        max_resolution,
        max_deviation,
        wall_line_count,
        small_area_width,
        infill_origin,
        skip_line_stitching,
        fill_gaps,
        connected_zigzags,
        use_endpieces,
        skip_some_zags,
        zag_skip_count,
        pocket_size);
    infill_comp.generate(
        skin_paths,
        skin_polygons,
        skin_lines,
        mesh.settings,
        gcode_layer.getLayerNr(),
        SectionType::SKIN,
        nullptr,
        nullptr,
        nullptr,
        small_areas_on_surface ? Polygons() : exposed_to_air);

    // add paths
    if (! skin_polygons.empty() || ! skin_lines.empty() || ! skin_paths.empty())
    {
        added_something = true;
        gcode_layer.setIsInside(true); // going to print stuff inside print object
        if (! skin_paths.empty())
        {
            // Add skin-walls a.k.a. skin-perimeters, skin-insets.
            const size_t skin_extruder_nr = mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr;
            if (extruder_nr == skin_extruder_nr)
            {
                constexpr bool retract_before_outer_wall = false;
                constexpr coord_t wipe_dist = 0;
                const ZSeamConfig z_seam_config(
                    mesh.settings.get<EZSeamType>("z_seam_type"),
                    mesh.getZSeamHint(),
                    mesh.settings.get<EZSeamCornerPrefType>("z_seam_corner"),
                    config.getLineWidth() * 2);
                InsetOrderOptimizer wall_orderer(
                    *this,
                    storage,
                    gcode_layer,
                    mesh.settings,
                    extruder_nr,
                    config,
                    config,
                    config,
                    config,
                    retract_before_outer_wall,
                    wipe_dist,
                    wipe_dist,
                    skin_extruder_nr,
                    skin_extruder_nr,
                    z_seam_config,
                    skin_paths);
                added_something |= wall_orderer.addToLayer();
            }
        }
        if (! skin_polygons.empty())
        {
            constexpr bool force_comb_retract = false;
            gcode_layer.addTravel(skin_polygons[0][0], force_comb_retract);
            gcode_layer.addPolygonsByOptimizer(skin_polygons, config);
        }

        if (monotonic)
        {
            const coord_t exclude_distance = config.getLineWidth() * 0.8;

            const AngleRadians monotonic_direction = AngleRadians(skin_angle);
            constexpr Ratio flow = 1.0_r;
            const coord_t max_adjacent_distance
                = config.getLineWidth()
                * 1.1; // Lines are considered adjacent if they are 1 line width apart, with 10% extra play. The monotonic order is enforced if they are adjacent.
            if (pattern == EFillMethod::GRID || pattern == EFillMethod::LINES || pattern == EFillMethod::TRIANGLES || pattern == EFillMethod::CUBIC
                || pattern == EFillMethod::TETRAHEDRAL || pattern == EFillMethod::QUARTER_CUBIC || pattern == EFillMethod::CUBICSUBDIV || pattern == EFillMethod::LIGHTNING)
            {
                gcode_layer.addLinesMonotonic(
                    area,
                    skin_lines,
                    config,
                    SpaceFillType::Lines,
                    monotonic_direction,
                    max_adjacent_distance,
                    exclude_distance,
                    mesh.settings.get<coord_t>("infill_wipe_dist"),
                    flow,
                    fan_speed);
            }
            else
            {
                const SpaceFillType space_fill_type = (pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines;
                constexpr coord_t wipe_dist = 0;
                gcode_layer.addLinesMonotonic(area, skin_lines, config, space_fill_type, monotonic_direction, max_adjacent_distance, exclude_distance, wipe_dist, flow, fan_speed);
            }
        }
        else
        {
            std::optional<Point> near_start_location;
            const EFillMethod pattern
                = (gcode_layer.getLayerNr() == 0) ? mesh.settings.get<EFillMethod>("top_bottom_pattern_0") : mesh.settings.get<EFillMethod>("top_bottom_pattern");
            if (pattern == EFillMethod::LINES || pattern == EFillMethod::ZIG_ZAG)
            { // update near_start_location to a location which tries to avoid seams in skin
                near_start_location = getSeamAvoidingLocation(area, skin_angle, gcode_layer.getLastPlannedPositionOrStartingPosition());
            }

            constexpr bool enable_travel_optimization = false;
            constexpr float flow = 1.0;
            if (pattern == EFillMethod::GRID || pattern == EFillMethod::LINES || pattern == EFillMethod::TRIANGLES || pattern == EFillMethod::CUBIC
                || pattern == EFillMethod::TETRAHEDRAL || pattern == EFillMethod::QUARTER_CUBIC || pattern == EFillMethod::CUBICSUBDIV || pattern == EFillMethod::LIGHTNING)
            {
                gcode_layer.addLinesByOptimizer(
                    skin_lines,
                    config,
                    SpaceFillType::Lines,
                    enable_travel_optimization,
                    mesh.settings.get<coord_t>("infill_wipe_dist"),
                    flow,
                    near_start_location,
                    fan_speed);
            }
            else
            {
                SpaceFillType space_fill_type = (pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines;
                constexpr coord_t wipe_dist = 0;
                gcode_layer.addLinesByOptimizer(skin_lines, config, space_fill_type, enable_travel_optimization, wipe_dist, flow, near_start_location, fan_speed);
            }
        }
    }
}

bool FffGcodeWriter::processIroning(
    const SliceDataStorage& storage,
    const SliceMeshStorage& mesh,
    const SliceLayer& layer,
    const GCodePathConfig& line_config,
    LayerPlan& gcode_layer) const
{
    bool added_something = false;
    const bool ironing_enabled = mesh.settings.get<bool>("ironing_enabled");
    const bool ironing_only_highest_layer = mesh.settings.get<bool>("ironing_only_highest_layer");
    if (ironing_enabled && (! ironing_only_highest_layer || mesh.layer_nr_max_filled_layer == gcode_layer.getLayerNr()))
    {
        // Since we are ironing after all the parts are completed, it believes that it is outside.
        // But the truth is that we are inside a part, so we need to change it before we do the ironing
        // See CURA-8615
        gcode_layer.setIsInside(true);
        added_something |= layer.top_surface.ironing(storage, mesh, line_config, gcode_layer, *this);
        gcode_layer.setIsInside(false);
    }
    return added_something;
}


bool FffGcodeWriter::addSupportToGCode(const SliceDataStorage& storage, LayerPlan& gcode_layer, const size_t extruder_nr) const
{
    bool support_added = false;
    if (! storage.support.generated || gcode_layer.getLayerNr() > storage.support.layer_nr_max_filled_layer)
    {
        return support_added;
    }

    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const size_t support_roof_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("support_roof_extruder_nr").extruder_nr;
    const size_t support_bottom_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("support_bottom_extruder_nr").extruder_nr;
    size_t support_infill_extruder_nr = (gcode_layer.getLayerNr() <= 0) ? mesh_group_settings.get<ExtruderTrain&>("support_extruder_nr_layer_0").extruder_nr
                                                                        : mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr;

    const SupportLayer& support_layer = storage.support.supportLayers[std::max(LayerIndex{ 0 }, gcode_layer.getLayerNr())];
    if (support_layer.support_bottom.empty() && support_layer.support_roof.empty() && support_layer.support_infill_parts.empty())
    {
        return support_added;
    }

    if (extruder_nr == support_roof_extruder_nr)
    {
        support_added |= addSupportRoofsToGCode(storage, support_layer.support_fractional_roof, gcode_layer.configs_storage.support_fractional_roof_config, gcode_layer);
    }
    if (extruder_nr == support_infill_extruder_nr)
    {
        support_added |= processSupportInfill(storage, gcode_layer);
    }
    if (extruder_nr == support_roof_extruder_nr)
    {
        support_added |= addSupportRoofsToGCode(
            storage,
            support_layer.support_roof.difference(support_layer.support_fractional_roof),
            gcode_layer.configs_storage.support_roof_config,
            gcode_layer);
    }
    if (extruder_nr == support_bottom_extruder_nr)
    {
        support_added |= addSupportBottomsToGCode(storage, gcode_layer);
    }
    return support_added;
}


bool FffGcodeWriter::processSupportInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer) const
{
    bool added_something = false;
    const SupportLayer& support_layer
        = storage.support.supportLayers[std::max(LayerIndex{ 0 }, gcode_layer.getLayerNr())]; // account for negative layer numbers for raft filler layers

    if (gcode_layer.getLayerNr() > storage.support.layer_nr_max_filled_layer || support_layer.support_infill_parts.empty())
    {
        return added_something;
    }

    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const size_t extruder_nr = (gcode_layer.getLayerNr() <= 0) ? mesh_group_settings.get<ExtruderTrain&>("support_extruder_nr_layer_0").extruder_nr
                                                               : mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr;
    const ExtruderTrain& infill_extruder = Application::getInstance().current_slice->scene.extruders[extruder_nr];

    coord_t default_support_line_distance = infill_extruder.settings.get<coord_t>("support_line_distance");

    // To improve adhesion for the "support initial layer" the first layer might have different properties
    if (gcode_layer.getLayerNr() == 0)
    {
        default_support_line_distance = infill_extruder.settings.get<coord_t>("support_initial_layer_line_distance");
    }

    const coord_t default_support_infill_overlap = infill_extruder.settings.get<coord_t>("infill_overlap_mm");

    // Helper to get the support infill angle
    const auto get_support_infill_angle = [](const SupportStorage& support_storage, const int layer_nr)
    {
        if (layer_nr <= 0)
        {
            // handle negative layer numbers
            const size_t divisor = support_storage.support_infill_angles_layer_0.size();
            const size_t index = ((layer_nr % divisor) + divisor) % divisor;
            return support_storage.support_infill_angles_layer_0.at(index);
        }
        return support_storage.support_infill_angles.at(static_cast<size_t>(layer_nr) % support_storage.support_infill_angles.size());
    };
    const AngleDegrees support_infill_angle = get_support_infill_angle(storage.support, gcode_layer.getLayerNr());

    constexpr size_t infill_multiplier = 1; // there is no frontend setting for this (yet)
    const size_t wall_line_count = infill_extruder.settings.get<size_t>("support_wall_count");
    const coord_t max_resolution = infill_extruder.settings.get<coord_t>("meshfix_maximum_resolution");
    const coord_t max_deviation = infill_extruder.settings.get<coord_t>("meshfix_maximum_deviation");
    coord_t default_support_line_width = infill_extruder.settings.get<coord_t>("support_line_width");
    if (gcode_layer.getLayerNr() == 0 && mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::RAFT)
    {
        default_support_line_width *= infill_extruder.settings.get<Ratio>("initial_layer_line_width_factor");
    }

    // Helper to get the support pattern
    const auto get_support_pattern = [](const EFillMethod pattern, const int layer_nr)
    {
        if (layer_nr <= 0 && (pattern == EFillMethod::LINES || pattern == EFillMethod::ZIG_ZAG))
        {
            return EFillMethod::GRID;
        }
        return pattern;
    };
    const EFillMethod support_pattern = get_support_pattern(infill_extruder.settings.get<EFillMethod>("support_pattern"), gcode_layer.getLayerNr());

    const auto zig_zaggify_infill = infill_extruder.settings.get<bool>("zig_zaggify_support");
    const auto skip_some_zags = infill_extruder.settings.get<bool>("support_skip_some_zags");
    const auto zag_skip_count = infill_extruder.settings.get<size_t>("support_zag_skip_count");

    // create a list of outlines and use PathOrderOptimizer to optimize the travel move
    PathOrderOptimizer<const SupportInfillPart*> island_order_optimizer_initial(gcode_layer.getLastPlannedPositionOrStartingPosition());
    PathOrderOptimizer<const SupportInfillPart*> island_order_optimizer(gcode_layer.getLastPlannedPositionOrStartingPosition());
    for (const SupportInfillPart& part : support_layer.support_infill_parts)
    {
        (part.use_fractional_config ? island_order_optimizer_initial : island_order_optimizer).addPolygon(&part);
    }
    island_order_optimizer_initial.optimize();
    island_order_optimizer.optimize();

    const auto support_connect_zigzags = infill_extruder.settings.get<bool>("support_connect_zigzags");
    const auto support_structure = infill_extruder.settings.get<ESupportStructure>("support_structure");
    const Point infill_origin;

    constexpr bool use_endpieces = true;
    constexpr coord_t pocket_size = 0;
    constexpr bool connect_polygons = false; // polygons are too distant to connect for sparse support
    bool need_travel_to_end_of_last_spiral = true;

    // Print the thicker infill lines first. (double or more layer thickness, infill combined with previous layers)
    for (const PathOrdering<const SupportInfillPart*>& path : ranges::views::concat(island_order_optimizer_initial.paths, island_order_optimizer.paths))
    {
        const SupportInfillPart& part = *path.vertices;
        const auto& configs = part.use_fractional_config ? gcode_layer.configs_storage.support_fractional_infill_config : gcode_layer.configs_storage.support_infill_config;

        // always process the wall overlap if walls are generated
        const int current_support_infill_overlap = (part.inset_count_to_generate > 0) ? default_support_infill_overlap : 0;

        // The support infill walls were generated separately, first. Always add them, regardless of how many densities we have.
        std::vector<VariableWidthLines> wall_toolpaths = part.wall_toolpaths;

        if (! wall_toolpaths.empty())
        {
            const GCodePathConfig& config = configs[0];
            constexpr bool retract_before_outer_wall = false;
            constexpr coord_t wipe_dist = 0;
            const ZSeamConfig z_seam_config(EZSeamType::SHORTEST, gcode_layer.getLastPlannedPositionOrStartingPosition(), EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE, false);
            InsetOrderOptimizer wall_orderer(
                *this,
                storage,
                gcode_layer,
                infill_extruder.settings,
                extruder_nr,
                config,
                config,
                config,
                config,
                retract_before_outer_wall,
                wipe_dist,
                wipe_dist,
                extruder_nr,
                extruder_nr,
                z_seam_config,
                wall_toolpaths);
            added_something |= wall_orderer.addToLayer();
        }

        if ((default_support_line_distance <= 0 && support_structure != ESupportStructure::TREE) || part.infill_area_per_combine_per_density.empty())
        {
            continue;
        }

        for (unsigned int combine_idx = 0; combine_idx < part.infill_area_per_combine_per_density[0].size(); ++combine_idx)
        {
            const coord_t support_line_width = default_support_line_width * (combine_idx + 1);

            Polygons support_polygons;
            std::vector<VariableWidthLines> wall_toolpaths_here;
            Polygons support_lines;
            const size_t max_density_idx = part.infill_area_per_combine_per_density.size() - 1;
            for (size_t density_idx = max_density_idx; (density_idx + 1) > 0; --density_idx)
            {
                if (combine_idx >= part.infill_area_per_combine_per_density[density_idx].size())
                {
                    continue;
                }

                const unsigned int density_factor = 2 << density_idx; // == pow(2, density_idx + 1)
                int support_line_distance_here
                    = (part.custom_line_distance > 0
                           ? part.custom_line_distance
                           : default_support_line_distance * density_factor); // the highest density infill combines with the next to create a grid with density_factor 1
                const int support_shift = support_line_distance_here / 2;
                if (part.custom_line_distance == 0 && (density_idx == max_density_idx || support_pattern == EFillMethod::CROSS || support_pattern == EFillMethod::CROSS_3D))
                {
                    support_line_distance_here /= 2;
                }
                const Polygons& area = Simplify(infill_extruder.settings).polygon(part.infill_area_per_combine_per_density[density_idx][combine_idx]);

                constexpr size_t wall_count = 0; // Walls are generated somewhere else, so their layers aren't vertically combined.
                const coord_t small_area_width = 0;
                constexpr bool skip_stitching = false;
                const bool fill_gaps = density_idx == 0; // Only fill gaps for one of the densities.
                Infill infill_comp(
                    support_pattern,
                    zig_zaggify_infill,
                    connect_polygons,
                    area,
                    support_line_width,
                    support_line_distance_here,
                    current_support_infill_overlap - (density_idx == max_density_idx ? 0 : wall_line_count * support_line_width),
                    infill_multiplier,
                    support_infill_angle,
                    gcode_layer.z + configs[combine_idx].z_offset,
                    support_shift,
                    max_resolution,
                    max_deviation,
                    wall_count,
                    small_area_width,
                    infill_origin,
                    skip_stitching,
                    fill_gaps,
                    support_connect_zigzags,
                    use_endpieces,
                    skip_some_zags,
                    zag_skip_count,
                    pocket_size);
                infill_comp.generate(
                    wall_toolpaths_here,
                    support_polygons,
                    support_lines,
                    infill_extruder.settings,
                    gcode_layer.getLayerNr(),
                    SectionType::SUPPORT,
                    storage.support.cross_fill_provider);
            }

            if (need_travel_to_end_of_last_spiral && infill_extruder.settings.get<bool>("magic_spiralize"))
            {
                if ((! wall_toolpaths.empty() || ! support_polygons.empty() || ! support_lines.empty()))
                {
                    int layer_nr = gcode_layer.getLayerNr();
                    if (layer_nr > (int)infill_extruder.settings.get<size_t>("initial_bottom_layers"))
                    {
                        // bit of subtlety here... support is being used on a spiralized model and to ensure the travel move from the end of the last spiral
                        // to the start of the support does not go through the model we have to tell the slicer what the current location of the nozzle is
                        // by adding a travel move to the end vertex of the last spiral. Of course, if the slicer could track the final location on the previous
                        // layer then this wouldn't be necessary but that's not done due to the multi-threading.
                        const Polygons* last_wall_outline = storage.spiralize_wall_outlines[layer_nr - 1];
                        if (last_wall_outline != nullptr)
                        {
                            gcode_layer.addTravel((*last_wall_outline)[0][storage.spiralize_seam_vertex_indices[layer_nr - 1]]);
                            need_travel_to_end_of_last_spiral = false;
                        }
                    }
                }
            }

            gcode_layer.setIsInside(false); // going to print stuff outside print object, i.e. support

            const bool alternate_inset_direction = infill_extruder.settings.get<bool>("material_alternate_walls");
            const bool alternate_layer_print_direction = alternate_inset_direction && gcode_layer.getLayerNr() % 2 == 1;

            if (! support_polygons.empty())
            {
                constexpr bool force_comb_retract = false;
                gcode_layer.addTravel(support_polygons[0][0], force_comb_retract);

                const ZSeamConfig& z_seam_config = ZSeamConfig();
                constexpr coord_t wall_0_wipe_dist = 0;
                constexpr bool spiralize = false;
                constexpr Ratio flow_ratio = 1.0_r;
                constexpr bool always_retract = false;
                const std::optional<Point> start_near_location = std::optional<Point>();

                gcode_layer.addPolygonsByOptimizer(
                    support_polygons,
                    configs[combine_idx],
                    z_seam_config,
                    wall_0_wipe_dist,
                    spiralize,
                    flow_ratio,
                    always_retract,
                    alternate_layer_print_direction,
                    start_near_location);
                added_something = true;
            }

            if (! support_lines.empty())
            {
                constexpr bool enable_travel_optimization = false;
                constexpr coord_t wipe_dist = 0;
                constexpr Ratio flow_ratio = 1.0;
                const std::optional<Point> near_start_location = std::optional<Point>();
                constexpr double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT;

                gcode_layer.addLinesByOptimizer(
                    support_lines,
                    configs[combine_idx],
                    (support_pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines,
                    enable_travel_optimization,
                    wipe_dist,
                    flow_ratio,
                    near_start_location,
                    fan_speed,
                    alternate_layer_print_direction);

                added_something = true;
            }

            // If we're printing with a support wall, that support wall generates gap filling as well.
            // If not, the pattern may still generate gap filling (if it's connected infill or zigzag). We still want to print those.
            if (wall_line_count == 0 || ! wall_toolpaths_here.empty())
            {
                const GCodePathConfig& config = configs[0];
                constexpr bool retract_before_outer_wall = false;
                constexpr coord_t wipe_dist = 0;
                constexpr coord_t simplify_curvature = 0;
                const ZSeamConfig z_seam_config(
                    EZSeamType::SHORTEST,
                    gcode_layer.getLastPlannedPositionOrStartingPosition(),
                    EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE,
                    simplify_curvature);
                InsetOrderOptimizer wall_orderer(
                    *this,
                    storage,
                    gcode_layer,
                    infill_extruder.settings,
                    extruder_nr,
                    config,
                    config,
                    config,
                    config,
                    retract_before_outer_wall,
                    wipe_dist,
                    wipe_dist,
                    extruder_nr,
                    extruder_nr,
                    z_seam_config,
                    wall_toolpaths_here);
                added_something |= wall_orderer.addToLayer();
            }
        }
    }

    return added_something;
}


bool FffGcodeWriter::addSupportRoofsToGCode(
    const SliceDataStorage& storage,
    const Polygons& support_roof_outlines,
    const GCodePathConfig& current_roof_config,
    LayerPlan& gcode_layer) const
{
    const SupportLayer& support_layer = storage.support.supportLayers[std::max(LayerIndex{ 0 }, gcode_layer.getLayerNr())];

    if (! storage.support.generated || gcode_layer.getLayerNr() > storage.support.layer_nr_max_filled_layer || support_layer.support_roof.empty())
    {
        return false; // No need to generate support roof if there's no support.
    }

    const size_t roof_extruder_nr = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("support_roof_extruder_nr").extruder_nr;
    const ExtruderTrain& roof_extruder = Application::getInstance().current_slice->scene.extruders[roof_extruder_nr];

    const EFillMethod pattern = roof_extruder.settings.get<EFillMethod>("support_roof_pattern");
    AngleDegrees fill_angle = 0;
    if (! storage.support.support_roof_angles.empty())
    {
        // handle negative layer numbers
        int divisor = static_cast<int>(storage.support.support_roof_angles.size());
        int index = ((gcode_layer.getLayerNr() % divisor) + divisor) % divisor;
        fill_angle = storage.support.support_roof_angles.at(index);
    }
    const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
    const bool connect_polygons = false; // connections might happen in mid air in between the infill lines
    constexpr coord_t support_roof_overlap = 0; // the roofs should never be expanded outwards
    constexpr size_t infill_multiplier = 1;
    constexpr coord_t extra_infill_shift = 0;
    const auto wall_line_count = roof_extruder.settings.get<size_t>("support_roof_wall_count");
    const coord_t small_area_width = roof_extruder.settings.get<coord_t>("min_even_wall_line_width") * 2; // Maximum width of a region that can still be filled with one wall.
    const Point infill_origin;
    constexpr bool skip_stitching = false;
    constexpr bool fill_gaps = true;
    constexpr bool use_endpieces = true;
    constexpr bool connected_zigzags = false;
    constexpr bool skip_some_zags = false;
    constexpr size_t zag_skip_count = 0;
    constexpr coord_t pocket_size = 0;
    const coord_t max_resolution = roof_extruder.settings.get<coord_t>("meshfix_maximum_resolution");
    const coord_t max_deviation = roof_extruder.settings.get<coord_t>("meshfix_maximum_deviation");

    coord_t support_roof_line_distance = roof_extruder.settings.get<coord_t>("support_roof_line_distance");
    const coord_t support_roof_line_width = roof_extruder.settings.get<coord_t>("support_roof_line_width");
    if (gcode_layer.getLayerNr() == 0 && support_roof_line_distance < 2 * support_roof_line_width)
    { // if roof is dense
        support_roof_line_distance *= roof_extruder.settings.get<Ratio>("initial_layer_line_width_factor");
    }

    Polygons infill_outline = support_roof_outlines;
    Polygons wall;
    // make sure there is a wall if this is on the first layer
    if (gcode_layer.getLayerNr() == 0)
    {
        wall = support_layer.support_roof.offset(-support_roof_line_width / 2);
        infill_outline = wall.offset(-support_roof_line_width / 2);
    }
    infill_outline = Simplify(roof_extruder.settings).polygon(infill_outline);

    Infill roof_computation(
        pattern,
        zig_zaggify_infill,
        connect_polygons,
        infill_outline,
        current_roof_config.getLineWidth(),
        support_roof_line_distance,
        support_roof_overlap,
        infill_multiplier,
        fill_angle,
        gcode_layer.z + current_roof_config.z_offset,
        extra_infill_shift,
        max_resolution,
        max_deviation,
        wall_line_count,
        small_area_width,
        infill_origin,
        skip_stitching,
        fill_gaps,
        connected_zigzags,
        use_endpieces,
        skip_some_zags,
        zag_skip_count,
        pocket_size);
    Polygons roof_polygons;
    std::vector<VariableWidthLines> roof_paths;
    Polygons roof_lines;
    roof_computation.generate(roof_paths, roof_polygons, roof_lines, roof_extruder.settings, gcode_layer.getLayerNr(), SectionType::SUPPORT);
    if ((gcode_layer.getLayerNr() == 0 && wall.empty()) || (gcode_layer.getLayerNr() > 0 && roof_paths.empty() && roof_polygons.empty() && roof_lines.empty()))
    {
        return false; // We didn't create any support roof.
    }
    gcode_layer.setIsInside(false); // going to print stuff outside print object, i.e. support
    if (gcode_layer.getLayerNr() == 0)
    {
        gcode_layer.addPolygonsByOptimizer(wall, current_roof_config);
    }
    if (! roof_polygons.empty())
    {
        constexpr bool force_comb_retract = false;
        gcode_layer.addTravel(roof_polygons[0][0], force_comb_retract);
        gcode_layer.addPolygonsByOptimizer(roof_polygons, current_roof_config);
    }
    if (! roof_paths.empty())
    {
        const GCodePathConfig& config = current_roof_config;
        constexpr bool retract_before_outer_wall = false;
        constexpr coord_t wipe_dist = 0;
        const ZSeamConfig z_seam_config(EZSeamType::SHORTEST, gcode_layer.getLastPlannedPositionOrStartingPosition(), EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE, false);

        InsetOrderOptimizer wall_orderer(
            *this,
            storage,
            gcode_layer,
            roof_extruder.settings,
            roof_extruder_nr,
            config,
            config,
            config,
            config,
            retract_before_outer_wall,
            wipe_dist,
            wipe_dist,
            roof_extruder_nr,
            roof_extruder_nr,
            z_seam_config,
            roof_paths);
        wall_orderer.addToLayer();
    }
    gcode_layer.addLinesByOptimizer(roof_lines, current_roof_config, (pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines);
    return true;
}

bool FffGcodeWriter::addSupportBottomsToGCode(const SliceDataStorage& storage, LayerPlan& gcode_layer) const
{
    const SupportLayer& support_layer = storage.support.supportLayers[std::max(LayerIndex{ 0 }, gcode_layer.getLayerNr())];

    if (! storage.support.generated || gcode_layer.getLayerNr() > storage.support.layer_nr_max_filled_layer || support_layer.support_bottom.empty())
    {
        return false; // No need to generate support bottoms if there's no support.
    }

    const size_t bottom_extruder_nr = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("support_bottom_extruder_nr").extruder_nr;
    const ExtruderTrain& bottom_extruder = Application::getInstance().current_slice->scene.extruders[bottom_extruder_nr];

    const EFillMethod pattern = bottom_extruder.settings.get<EFillMethod>("support_bottom_pattern");
    AngleDegrees fill_angle = 0;
    if (! storage.support.support_bottom_angles.empty())
    {
        // handle negative layer numbers
        int divisor = static_cast<int>(storage.support.support_bottom_angles.size());
        int index = ((gcode_layer.getLayerNr() % divisor) + divisor) % divisor;
        fill_angle = storage.support.support_bottom_angles.at(index);
    }
    const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
    constexpr bool connect_polygons = false; // Keep the same as roof, also does make a bit less sense when support infill is < 100% or support walls are set to > 0.
    constexpr coord_t support_bottom_overlap = 0; // the bottoms should never be expanded outwards
    constexpr size_t infill_multiplier = 1;
    constexpr coord_t extra_infill_shift = 0;
    const auto wall_line_count = bottom_extruder.settings.get<size_t>("support_bottom_wall_count");
    const coord_t small_area_width = bottom_extruder.settings.get<coord_t>("min_even_wall_line_width") * 2; // Maximum width of a region that can still be filled with one wall.

    const Point infill_origin;
    constexpr bool skip_stitching = false;
    constexpr bool fill_gaps = true;
    constexpr bool use_endpieces = true;
    constexpr bool connected_zigzags = false;
    constexpr bool skip_some_zags = false;
    constexpr int zag_skip_count = 0;
    constexpr coord_t pocket_size = 0;
    const coord_t max_resolution = bottom_extruder.settings.get<coord_t>("meshfix_maximum_resolution");
    const coord_t max_deviation = bottom_extruder.settings.get<coord_t>("meshfix_maximum_deviation");

    const coord_t support_bottom_line_distance = bottom_extruder.settings.get<coord_t>(
        "support_bottom_line_distance"); // note: no need to apply initial line width factor; support bottoms cannot exist on the first layer
    Infill bottom_computation(
        pattern,
        zig_zaggify_infill,
        connect_polygons,
        support_layer.support_bottom,
        gcode_layer.configs_storage.support_bottom_config.getLineWidth(),
        support_bottom_line_distance,
        support_bottom_overlap,
        infill_multiplier,
        fill_angle,
        gcode_layer.z,
        extra_infill_shift,
        max_resolution,
        max_deviation,
        wall_line_count,
        small_area_width,
        infill_origin,
        skip_stitching,
        fill_gaps,
        connected_zigzags,
        use_endpieces,
        skip_some_zags,
        zag_skip_count,
        pocket_size);
    Polygons bottom_polygons;
    std::vector<VariableWidthLines> bottom_paths;
    Polygons bottom_lines;
    bottom_computation.generate(bottom_paths, bottom_polygons, bottom_lines, bottom_extruder.settings, gcode_layer.getLayerNr(), SectionType::SUPPORT);
    if (bottom_paths.empty() && bottom_polygons.empty() && bottom_lines.empty())
    {
        return false;
    }
    gcode_layer.setIsInside(false); // going to print stuff outside print object, i.e. support
    if (! bottom_polygons.empty())
    {
        constexpr bool force_comb_retract = false;
        gcode_layer.addTravel(bottom_polygons[0][0], force_comb_retract);
        gcode_layer.addPolygonsByOptimizer(bottom_polygons, gcode_layer.configs_storage.support_bottom_config);
    }
    if (! bottom_paths.empty())
    {
        const GCodePathConfig& config = gcode_layer.configs_storage.support_bottom_config;
        constexpr bool retract_before_outer_wall = false;
        constexpr coord_t wipe_dist = 0;
        const ZSeamConfig z_seam_config(EZSeamType::SHORTEST, gcode_layer.getLastPlannedPositionOrStartingPosition(), EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE, false);

        InsetOrderOptimizer wall_orderer(
            *this,
            storage,
            gcode_layer,
            bottom_extruder.settings,
            bottom_extruder_nr,
            config,
            config,
            config,
            config,
            retract_before_outer_wall,
            wipe_dist,
            wipe_dist,
            bottom_extruder_nr,
            bottom_extruder_nr,
            z_seam_config,
            bottom_paths);
        wall_orderer.addToLayer();
    }
    gcode_layer.addLinesByOptimizer(
        bottom_lines,
        gcode_layer.configs_storage.support_bottom_config,
        (pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines);
    return true;
}

void FffGcodeWriter::setExtruder_addPrime(const SliceDataStorage& storage, LayerPlan& gcode_layer, const size_t extruder_nr) const
{
    const size_t previous_extruder = gcode_layer.getExtruder();
    const bool extruder_changed = gcode_layer.setExtruder(extruder_nr);

    if (extruder_changed)
    {
        if (extruder_prime_layer_nr[extruder_nr] == gcode_layer.getLayerNr())
        {
            const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];

            // We always prime an extruder, but whether it will be a prime blob/poop depends on if prime blob is enabled.
            // This is decided in GCodeExport::writePrimeTrain().
            if (train.settings.get<bool>("prime_blob_enable")) // Don't travel to the prime-blob position if not enabled though.
            {
                bool prime_pos_is_abs = train.settings.get<bool>("extruder_prime_pos_abs");
                Point prime_pos = Point(train.settings.get<coord_t>("extruder_prime_pos_x"), train.settings.get<coord_t>("extruder_prime_pos_y"));
                gcode_layer.addTravel(prime_pos_is_abs ? prime_pos : gcode_layer.getLastPlannedPositionOrStartingPosition() + prime_pos);
                gcode_layer.planPrime();
            }
            else
            {
                // Otherwise still prime, but don't do any other travels.
                gcode_layer.planPrime(0.0);
            }
        }

        if (! gcode_layer.getSkirtBrimIsPlanned(extruder_nr))
        {
            processSkirtBrim(storage, gcode_layer, extruder_nr, gcode_layer.getLayerNr());
        }
    }

    addPrimeTower(storage, gcode_layer, previous_extruder);
}

void FffGcodeWriter::addPrimeTower(const SliceDataStorage& storage, LayerPlan& gcode_layer, const size_t prev_extruder) const
{
    if (! Application::getInstance().current_slice->scene.current_mesh_group->settings.get<bool>("prime_tower_enable"))
    {
        return;
    }

    storage.primeTower.addToGcode(storage, gcode_layer, prev_extruder, gcode_layer.getExtruder());
}

void FffGcodeWriter::finalize()
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    if (mesh_group_settings.get<bool>("machine_heated_bed"))
    {
        gcode.writeBedTemperatureCommand(0); // Cool down the bed (M140).
        // Nozzles are cooled down automatically after the last time they are used (which might be earlier than the end of the print).
    }
    if (mesh_group_settings.get<bool>("machine_heated_build_volume") && mesh_group_settings.get<Temperature>("build_volume_temperature") != 0)
    {
        gcode.writeBuildVolumeTemperatureCommand(0); // Cool down the build volume.
    }

    const Duration print_time = gcode.getSumTotalPrintTimes();
    std::vector<double> filament_used;
    std::vector<std::string> material_ids;
    std::vector<bool> extruder_is_used;
    const Scene& scene = Application::getInstance().current_slice->scene;
    for (size_t extruder_nr = 0; extruder_nr < scene.extruders.size(); extruder_nr++)
    {
        filament_used.emplace_back(gcode.getTotalFilamentUsed(extruder_nr));
        material_ids.emplace_back(scene.extruders[extruder_nr].settings.get<std::string>("material_guid"));
        extruder_is_used.push_back(gcode.getExtruderIsUsed(extruder_nr));
    }
    std::string prefix = gcode.getFileHeader(extruder_is_used, &print_time, filament_used, material_ids);
    if (! Application::getInstance().communication->isSequential())
    {
        Application::getInstance().communication->sendGCodePrefix(prefix);
        Application::getInstance().communication->sendSliceUUID(slice_uuid);
    }
    else
    {
        spdlog::info("Gcode header after slicing: {}", prefix);
    }
    if (mesh_group_settings.get<bool>("acceleration_enabled"))
    {
        gcode.writePrintAcceleration(mesh_group_settings.get<Acceleration>("machine_acceleration"));
        gcode.writeTravelAcceleration(mesh_group_settings.get<Acceleration>("machine_acceleration"));
    }
    if (mesh_group_settings.get<bool>("jerk_enabled"))
    {
        gcode.writeJerk(mesh_group_settings.get<Velocity>("machine_max_jerk_xy"));
    }

    const auto end_gcode = mesh_group_settings.get<std::string>("machine_end_gcode");

    if (end_gcode.length() > 0 && mesh_group_settings.get<bool>("relative_extrusion"))
    {
        gcode.writeExtrusionMode(false); // ensure absolute extrusion mode is set before the end gcode
    }
    gcode.finalize(end_gcode.c_str());

    // set extrusion mode back to "normal"
    gcode.resetExtrusionMode();

    for (size_t e = 0; e < Application::getInstance().current_slice->scene.extruders.size(); e++)
    {
        gcode.writeTemperatureCommand(e, 0, false);
    }

    gcode.writeComment("End of Gcode");
    /*
    the profile string below can be executed since the M25 doesn't end the gcode on an UMO and when printing via USB.
    gcode.writeCode("M25 ;Stop reading from this point on.");
    gcode.writeComment("Cura profile string:");
    gcode.writeComment(FffProcessor::getInstance()->getAllLocalSettingsString() + FffProcessor::getInstance()->getProfileString());
    */
}


} // namespace cura
