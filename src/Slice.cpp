//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ExtruderTrain.h"
#include "Slice.h"
#include "utils/logoutput.h"

namespace cura
{

Slice::Slice(const size_t num_mesh_groups)
: scene(num_mesh_groups)
{}

void Slice::compute()
{
    logWarning("%s", scene.getAllSettingsString().c_str());
    for (std::vector<MeshGroup>::iterator mesh_group = scene.mesh_groups.begin(); mesh_group != scene.mesh_groups.end(); mesh_group++)
    {
        scene.current_mesh_group = mesh_group;
        for (ExtruderTrain& extruder : scene.extruders)
        {
            extruder.settings.setParent(&scene.current_mesh_group->settings);
        }
        scene.processMeshGroup(*mesh_group);
    }
}

void Slice::reset()
{
    scene.extruders.clear();
    scene.mesh_groups.clear();
    scene.settings = Settings();
}

}