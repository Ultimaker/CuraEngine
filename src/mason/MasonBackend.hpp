/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef INCLUDED_MASON_MASON_BACKEND_HPP
#define INCLUDED_MASON_MASON_BACKEND_HPP

#include "../gcodeExport.h"
#include "../MeshGroup.h"
#include "../settings/settings.h"

#include "PlanToGcode.hpp"
#include "PrintPlan.hpp"
#include "VolumeStore.hpp"
#include "WirePlan.hpp"

namespace cura {
namespace mason {

class MasonBackend {
public:
    MasonBackend();
    
    void process(const SettingsBaseVirtual *settings, const MeshGroup *mesh_group, GCodeExport *gcode_out);
    
private:
    void createWirePlan();
    void createPrintPlan();

    VolumeStore m_volume_store;
    WirePlan m_wire_plan;
    PrintPlan m_print_plan;
    PlanToGcode m_plan_to_gcode;
};

}
}

#endif
