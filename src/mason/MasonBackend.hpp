/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef INCLUDED_MASON_MASON_BACKEND_HPP
#define INCLUDED_MASON_MASON_BACKEND_HPP

#include "BuildPlan.hpp"
#include "BuildPlanner.hpp"
#include "GCodeExport.hpp"
#include "Mesh.hpp"
#include "PlanToGcode.hpp"
#include "Settings.hpp"

namespace mason {

class MasonBackend {
public:
    MasonBackend();
    
    void process(const SettingsBaseVirtual *settings, const MeshGroup *mesh_group, cura::GCodeExport *gcode_out);
    
private:
    VolumeStore m_target_volume;
    BuildPlan m_build_plan;
    BuildPlanner m_build_planner;
    PlanToGcode m_plan_to_gcode;
    GCodeExport m_gcode_out;
};

}

#endif
