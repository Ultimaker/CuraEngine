/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef INCLUDED_MASON_MASON_BACKEND_HPP
#define INCLUDED_MASON_MASON_BACKEND_HPP

#include "GCodeExport.hpp"
#include "Mesh.hpp"
#include "PlanToGcode.hpp"
#include "PrintPlan.hpp"
#include "Settings.hpp"
#include "VolumeStore.hpp"
#include "WirePlan.hpp"

namespace mason {

class MasonBackend {
public:
    MasonBackend();
    
    void process(const SettingsBaseVirtual *settings, const MeshGroup *mesh_group, cura::GCodeExport *gcode_out);
    
private:
    void createWirePlan();
    void createPrintPlan();

    VolumeStore m_volume_store;
    WirePlan m_wire_plan;
    PrintPlan m_print_plan;
    PlanToGcode m_plan_to_gcode;
    GCodeExport m_gcode_out;
};

}

#endif
