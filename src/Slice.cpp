// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "Slice.h"

#include <spdlog/spdlog.h>
#ifdef SENTRY_URL
#include <sentry.h>
#endif

#include "ExtruderTrain.h"
#include "slice_data/SliceDataStorage.h"

namespace cura
{

Slice::Slice(const size_t num_mesh_groups)
    : scene(num_mesh_groups)
{
}

void Slice::compute()
{
    spdlog::info("All settings: {}", scene.getAllSettingsString());
#ifdef SENTRY_URL
    {
        sentry_set_tag("cura.machine_name", scene.settings.get<std::string>("machine_name").c_str());
    }
#endif

    std::vector<std::shared_ptr<SliceDataStorage>> storages;
    for (const MeshGroup& mesh_group : scene.mesh_groups)
    {
        storages.push_back(std::make_shared<SliceDataStorage>(mesh_group.settings));
    }

    for (auto [mesh_group_index, mesh_group] : scene.mesh_groups | ranges::views::enumerate)
    {
        scene.current_mesh_group = std::next(scene.mesh_groups.begin(), mesh_group_index);
        for (ExtruderTrain& extruder : scene.extruders)
        {
            extruder.settings_.setParent(&mesh_group.settings);
        }
        Scene::processMeshGroup(mesh_group, *storages.at(mesh_group_index));
    }
}

void Slice::reset()
{
    scene.extruders.clear();
    scene.mesh_groups.clear();
    scene.settings = Settings();
}

} // namespace cura
