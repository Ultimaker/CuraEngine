/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#include "MasonBackend.hpp"

namespace cura {
namespace mason {

MasonBackend::MasonBackend()
{
}

void MasonBackend::process(const SettingsBaseVirtual *settings, const MeshGroup *mesh_group, GCodeExport *gcode_out)
{
    gcode_out->preSetup(mesh_group);

    m_volume_store.addMeshGroup(mesh_group);
    
    m_plan_to_gcode.process(settings,gcode_out);
}

}
}
