// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "Slice.h"

#include <spdlog/spdlog.h>
#ifdef SENTRY_URL
#include <sentry.h>
#endif

#include "Application.h"
#include "ExtruderTrain.h"
#include "FffProcessor.h"
#include "communication/Communication.h"
#include "progress/Progress.h"
#include "slice_data/MeshGroupSliceData.h"
#include "utils/gettime.h"

namespace cura
{

Slice::Slice(const size_t num_mesh_groups)
    : scene(num_mesh_groups)
    , slice_data_(num_mesh_groups)
{
}

void Slice::activateMeshGroup(const Settings& mesh_group_settings)
{
    for (ExtruderTrain& extruder : scene.extruders)
    {
        extruder.settings_.setParent(&mesh_group_settings);
    }
}

bool Slice::preProcessMeshGroup(MeshGroupSliceData& storage)
{
    FffProcessor* fff_processor = FffProcessor::getInstance();
    fff_processor->time_keeper.restart();

    TimeKeeper time_keeper_total;

    bool empty = true;
    for (const Mesh& mesh : storage.mesh_group_.meshes)
    {
        if (! mesh.settings_.get<bool>("infill_mesh") && ! mesh.settings_.get<bool>("anti_overhang_mesh"))
        {
            empty = false;
            break;
        }
    }

    if (empty)
    {
        Progress::messageProgress(Progress::Stage::FINISH, 1, 1); // 100% on this meshgroup
        spdlog::info("Pre-process duration: {:03.3f}s", time_keeper_total.restart());
        return false;
    }

    return fff_processor->polygon_generator.generateAreas(storage, &storage.mesh_group_, fff_processor->time_keeper);
}

void Slice::compute()
{
    spdlog::info("All settings: {}", scene.getAllSettingsString());
#ifdef SENTRY_URL
    {
        sentry_set_tag("cura.machine_name", scene.settings.get<std::string>("machine_name").c_str());
    }
#endif

    // First, create all the mesh groups slice data and start pre-processing them
    size_t index = 0;
    std::map<std::shared_ptr<MeshGroupSliceData>, bool> needs_processing;
    for (MeshGroup& mesh_group : scene.mesh_groups)
    {
        std::shared_ptr<MeshGroupSliceData> mesh_group_data = std::make_shared<MeshGroupSliceData>(mesh_group);
        slice_data_[index++] = mesh_group_data;
        activateMeshGroup(mesh_group.settings);
        needs_processing[mesh_group_data] = preProcessMeshGroup(*mesh_group_data);
    }

    // Now all the mesh groups have been pre-processed, we have enough data to finalize them. This is required so that
    // when processing the early groups, we have enough information over the other groups to make smart decisions.
    for (const std::shared_ptr<MeshGroupSliceData>& mesh_group_data : slice_data_)
    {
        if (needs_processing[mesh_group_data])
        {
            activateMeshGroup(mesh_group_data->settings_);
            processMeshGroup(*mesh_group_data);
        }
    }
}

void Slice::reset()
{
    scene.extruders.clear();
    scene.mesh_groups.clear();
    scene.settings = Settings();
}

void Slice::processMeshGroup(MeshGroupSliceData& storage)
{
    FffProcessor* fff_processor = FffProcessor::getInstance();
    fff_processor->time_keeper.restart();

    TimeKeeper time_keeper_total;

    Progress::messageProgressStage(Progress::Stage::EXPORT, &fff_processor->time_keeper);
    fff_processor->gcode_writer.writeGCode(storage, fff_processor->time_keeper);

    Progress::messageProgress(Progress::Stage::FINISH, 1, 1); // 100% on this meshgroup
    Application::getInstance().communication_->flushGCode();
    Application::getInstance().communication_->sendOptimizedLayerData();
    spdlog::info("Process duration: {:03.3f}s", time_keeper_total.restart());
}

} // namespace cura
