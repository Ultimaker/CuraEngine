// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "Scene.h"

#include <algorithm>

#include <fmt/chrono.h>
#include <spdlog/spdlog.h>

#include "Application.h"
#include "FffProcessor.h"
#include "Mesh.h"
#include "SliceDataStorage.h"
#include "communication/Communication.h"
#include "progress/Progress.h"

namespace curaengine
{
using namespace progress;
using namespace types;
using namespace types::geometry;
using namespace utils;

Scene::Scene()
{
}

const std::string Scene::getAllSettingsString() const
{
    std::stringstream output;
    output << settings.getAllSettingsString(); // Global settings.

    // Per-extruder settings.
    for (size_t extruder_nr = 0; extruder_nr < extruders.size(); extruder_nr++)
    {
        output << " -e" << extruder_nr << extruders[extruder_nr].settings_.getAllSettingsString();
    }

    for (size_t mesh_group_index = 0; mesh_group_index < mesh_groups_.size(); mesh_group_index++)
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
        const MeshGroup::Ptr& mesh_group = mesh_groups_[mesh_group_index];
        output << mesh_group->settings.getAllSettingsString();

        // Per-object settings.
        for (size_t mesh_index = 0; mesh_index < mesh_group->meshes.size(); mesh_index++)
        {
            const Mesh& mesh = mesh_group->meshes[mesh_index];
            output << " -e" << mesh.settings_.get<size_t>("extruder_nr") << " -l \"" << mesh_index << "\"" << mesh.settings_.getAllSettingsString();
        }
    }
    output << "\n";

    return output.str();
}

void Scene::processMeshGroup(MeshGroup& mesh_group)
{
    FffProcessor* fff_processor = FffProcessor::getInstance();
    fff_processor->time_keeper.restart();

    TimeKeeper time_keeper_total;

    bool empty = true;
    for (Mesh& mesh : mesh_group.meshes)
    {
        if (mesh.isModelMesh())
        {
            empty = false;
            break;
        }
    }
    if (empty)
    {
        Progress::messageProgress(Progress::Stage::Finish, 1, 1); // 100% on this meshgroup
        spdlog::info("Total time elapsed {}", std::chrono::duration<double>(time_keeper_total.restart()));
        return;
    }

    SliceDataStorage storage;
    if (! fff_processor->polygon_generator.generateAreas(storage, &mesh_group, fff_processor->time_keeper))
    {
        return;
    }

    Progress::messageProgressStage(Progress::Stage::Export, &fff_processor->time_keeper);
    fff_processor->gcode_writer.writeGCode(storage, fff_processor->time_keeper);

    Progress::messageProgress(Progress::Stage::Finish, 1, 1); // 100% on this meshgroup
    Application::getInstance().communication_->sendOptimizedLayerData();
    spdlog::info("Total time elapsed {}\n", std::chrono::duration<double>(time_keeper_total.restart()));
}

void Scene::appendMeshGroup(const std::shared_ptr<MeshGroup>& mesh_group)
{
    mesh_group->settings.setParent(&settings);
    mesh_groups_.push_back(mesh_group);
}

void Scene::clearMeshGroups()
{
    mesh_groups_.clear();
}

const std::vector<std::shared_ptr<MeshGroup>>& Scene::getMeshGroups() const
{
    return mesh_groups_;
}

const std::shared_ptr<MeshGroup>& Scene::getMeshGroup(const size_t index) const
{
    return mesh_groups_.at(index);
}

bool Scene::isCurrentMeshGroupFirst() const
{
    return ! mesh_groups_.empty() && mesh_groups_.front() == current_mesh_group;
}

std::shared_ptr<MeshGroup> Scene::getPreviousMeshGroup(const std::shared_ptr<MeshGroup>& mesh_group) const
{
    const auto iterator = std::find(mesh_groups_.begin(), mesh_groups_.end(), mesh_group);
    if (iterator == mesh_groups_.end() || iterator == mesh_groups_.begin())
    {
        return nullptr;
    }
    return *(iterator - 1);
}

} // namespace curaengine
