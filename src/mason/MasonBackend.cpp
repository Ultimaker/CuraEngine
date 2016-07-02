/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#include "MasonBackend.hpp"

#include "Point3.hpp"
#include "Time.hpp"

namespace mason {

MasonBackend::MasonBackend()
{
}

void MasonBackend::process(const SettingsStore *settings, const MeshGroup *mesh_group, cura::GCodeExport *gcode_out)
{
    // GCode generation proceeds in several stages:
    // 1. Model is sliced into thin slices to make a volumetric representation of the object (VolumeStore).
    // 2. The model is converted to a wire plan.  These sub-stages can be interleaved.
    // 2a. The VolumeStore is split into annotated pieces that describe the roles of various subvolumes.
    //     This does things like idetify where the shell ends, areas of overhang, etc.  This stage does
    //     not modify the volume, only subdivide it.
    // 2b. The annotated VolumeStore is modified to add things like support, identify bridged areas, deal
    //     with too small regions, etc.  This stage modifies the volume to reflect what will actually be printed.
    //     Special regions may be created that have pre-calculated wire sequences.
    // 2c. The modified VolumeStore is filled with wires and wire sequences (indivisible sequences of wires).
    //     Typical wires specify a w x l x d box that should be filled.  Some of the box might be filled
    //     before the wire is printed.  Flow rate is adjusted to make this work.
    // 3. The wire plan is converted to a tool head plan.
    // 4. The tool head plan is converted to GCode.

    m_gcode_out.setGCodeExporter(gcode_out);
    m_gcode_out.preSetup(mesh_group);

    TimeKeeper part_timer;
    m_target_volume.addMeshGroup(mesh_group);
    std::cout << "target_volume time " << part_timer.restart() << std::endl;

    m_build_plan.settings = settings;
    m_build_plan.target = &m_target_volume;
    m_build_planner.process(&m_build_plan);
    std::cout << "build_planner time " << part_timer.restart() << std::endl;

    m_plan_to_gcode.process(settings, &m_build_plan.print_plan, &m_gcode_out);
    std::cout << "plan_to_gcode time " << part_timer.restart() << std::endl;
}

}
