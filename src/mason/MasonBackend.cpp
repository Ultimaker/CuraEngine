/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#include "MasonBackend.hpp"

#include "Point3.hpp"

namespace mason {

MasonBackend::MasonBackend()
{
}

void MasonBackend::createWirePlan()
{
    Wire wire;
    Point3 start_pt(mmToInt(150.0),mmToInt(140.0),mmToInt(0.25));
    Point3 end_pt(mmToInt(170.0),mmToInt(140.0),mmToInt(0.25));
    wire.pt0 = start_pt;
    wire.pt1 = end_pt;
    wire.width = mmToInt(0.5);
    wire.height = mmToInt(0.25);
    m_wire_plan.addWire(wire);
}

void MasonBackend::createPrintPlan()
{
    std::shared_ptr<PrintPlanElement> elem;

    size_t num_layers = m_wire_plan.numLayers();
    for (size_t layer_idx=0U; layer_idx!=num_layers; ++layer_idx) {
        const WireLayer &layer = m_wire_plan.getLayer(layer_idx);

        size_t num_wires = layer.wires.size();
        for (size_t wire_idx=0U; wire_idx!=num_wires; ++wire_idx) {
            const Wire &wire = layer.wires[wire_idx];
            
            elem.reset(new HeadMove(wire.pt0,30.0));
            m_print_plan.addElement(elem);
            elem.reset(new ExtrudedSegment(wire.pt0,wire.pt1,30.0,intToMm(wire.width)*intToMm(wire.height)));
            m_print_plan.addElement(elem);
        }
    }
}

void MasonBackend::process(const SettingsBaseVirtual *settings, const MeshGroup *mesh_group, cura::GCodeExport *gcode_out)
{
    // GCode generation proceeds in several stages:
    //*1. Model is sliced into thin slices to make a volumetric representation of the object (VolumeStore).
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
    //*3. The wire plan is converted to a tool head plan.
    //*4. The tool head plan is converted to GCode.

    m_gcode_out.setGCodeExporter(gcode_out);
    m_gcode_out.preSetup(mesh_group);

    m_volume_store.addMeshGroup(mesh_group);

    createWirePlan();
    createPrintPlan();
    
    m_plan_to_gcode.process(settings, &m_print_plan, &m_gcode_out);
}

}
