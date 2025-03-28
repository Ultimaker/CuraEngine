// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "Scene.h"

#include <Slice.h>
#include <utils/polygonUtils.h>

#include <range/v3/algorithm/find_if.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/remove.hpp>
#include <spdlog/spdlog.h>

#include "Application.h"
#include "FffProcessor.h" //To start a slice.
#include "communication/Communication.h" //To flush g-code and layer view when we're done.
#include "progress/Progress.h"
#include "sliceDataStorage.h"
#include "utils/ExtrudersSet.h"

namespace cura
{

Scene::Scene(const size_t num_mesh_groups)
    : mesh_groups(num_mesh_groups)
    , current_mesh_group(&mesh_groups.front())
{
    for (MeshGroup& mesh_group : mesh_groups)
    {
        mesh_group.settings.setParent(&settings);
    }
}

const std::string Scene::getAllSettingsString() const
{
    std::stringstream output;
    output << settings.getAllSettingsString(); // Global settings.

    // Per-extruder settings.
    for (size_t extruder_nr = 0; extruder_nr < extruders_.size(); extruder_nr++)
    {
        output << " -e" << extruder_nr << extruders_[extruder_nr].settings_.getAllSettingsString();
    }

    for (size_t mesh_group_index = 0; mesh_group_index < mesh_groups.size(); mesh_group_index++)
    {
        if (mesh_group_index == 0)
        {
            output << " -g";
        }
        else
        {
            output << " --next";
        }

        // Per-mesh-group settings.
        const MeshGroup& mesh_group = mesh_groups[mesh_group_index];
        output << mesh_group.settings.getAllSettingsString();

        // Per-object settings.
        for (size_t mesh_index = 0; mesh_index < mesh_group.meshes.size(); mesh_index++)
        {
            const Mesh& mesh = mesh_group.meshes[mesh_index];
            output << " -e" << mesh.settings_.get<size_t>("extruder_nr") << " -l \"" << mesh_index << "\"" << mesh.settings_.getAllSettingsString();
        }
    }
    output << "\n";

    return output.str();
}

void Scene::processMeshGroup(MeshGroup& mesh_group)
{
    current_mesh_group = &mesh_group;

    FffProcessor* fff_processor = FffProcessor::getInstance();
    fff_processor->time_keeper.restart();

    TimeKeeper time_keeper_total;

    bool empty = true;
    for (Mesh& mesh : mesh_group.meshes)
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
        spdlog::info("Total time elapsed {:03.3f}s", time_keeper_total.restart());
        return;
    }

    current_slice_data_ = std::make_shared<SliceDataStorage>();
    if (! fff_processor->polygon_generator.generateAreas(*current_slice_data_, &mesh_group, fff_processor->time_keeper))
    {
        return;
    }

    Progress::messageProgressStage(Progress::Stage::EXPORT, &fff_processor->time_keeper);
    fff_processor->gcode_writer.writeGCode(*current_slice_data_, fff_processor->time_keeper);

    Progress::messageProgress(Progress::Stage::FINISH, 1, 1); // 100% on this meshgroup
    Application::getInstance().communication_->flushGCode();
    Application::getInstance().communication_->sendOptimizedLayerData();
    current_slice_data_.reset();
    spdlog::info("Total time elapsed {:03.3f}s\n", time_keeper_total.restart());
}

const ExtruderTrain& Scene::getExtruder(const ExtruderNumber extruder_nr) const
{
    return *ranges::find_if(
        extruders_,
        [extruder_nr](const ExtruderTrain& extruder)
        {
            return extruder.extruder_nr_ == extruder_nr;
        });
}

Scene& Scene::getCurrent()
{
    return Application::getInstance().current_slice_->scene;
}

} // namespace cura
