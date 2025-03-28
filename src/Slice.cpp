// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "Slice.h"

#include <spdlog/spdlog.h>
#ifdef SENTRY_URL
#include <sentry.h>
#endif

#include "ExtruderTrain.h"

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

    for (MeshGroup& mesh_group : scene.mesh_groups)
    {
        for (ExtruderTrain& extruder : scene.extruders_)
        {
            extruder.settings_.setParent(&mesh_group.settings);
        }
        scene.processMeshGroup(mesh_group);
    }
}

void Slice::reset()
{
    scene.extruders_.clear();
    scene.mesh_groups.clear();
    scene.settings = Settings();
}

} // namespace cura
