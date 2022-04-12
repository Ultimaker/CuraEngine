//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "LightningSupport.h"

#include "Application.h" //To get settings.
#include "ExtruderTrain.h"
#include "Slice.h"
#include "sliceDataStorage.h"
#include "progress/Progress.h"
#include "settings/EnumSettings.h"
#include "settings/types/Angle.h"
#include "settings/types/Ratio.h"
#include "utils/algorithm.h"
#include "utils/IntPoint.h"
#include "utils/logoutput.h"
#include "utils/math.h"

//#include "utils/polygon.h"
//#include "utils/polygonUtils.h"

//#include <mutex>

namespace cura
{

LightningSupport::LightningSupport()
{
}

void LightningSupport::generateSupportAreas(SliceDataStorage& storage)
{
    const Settings& group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const bool global_use_tree_support =
        group_settings.get<bool>("support_enable") && group_settings.get<ESupportStructure>("support_structure") == ESupportStructure::LIGHTNING;
    if (! global_use_tree_support)
    {
        return;
    }
    const bool any_mesh_uses = std::any_of
        (
            storage.meshes.cbegin(),
            storage.meshes.cend(),
            [](const SliceMeshStorage& m)
            {
                return m.settings.get<bool>("support_enable") && m.settings.get<ESupportStructure>("support_structure") == ESupportStructure::LIGHTNING;
            }
        );
    if (! any_mesh_uses)
    {
        return;
    }

    std::for_each(storage.meshes.begin(), storage.meshes.end(), [this](SliceMeshStorage& mesh){ generateSupportForMesh(mesh); });
}

void LightningSupport::generateSupportForMesh(SliceMeshStorage& mesh)
{
    if (mesh.settings.get<ESupportStructure>("support_structure") != ESupportStructure::LIGHTNING)
    {
        return;
    }

    // TODO!
}

} // namespace cura
