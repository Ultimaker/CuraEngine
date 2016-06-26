/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#include "MasonBackend.hpp"

#include "Point3.hpp"

namespace mason {

MasonBackend::MasonBackend()
{
}

void MasonBackend::createWirePlan()
{
    coord_t top_z = mmToInt(0.25);
    coord_t bot_z = mmToInt(0.0);
    coord_t mid_z = (top_z + bot_z)/2;
    coord_t height = top_z - bot_z;

    size_t layer_idx = m_volume_store.getLayerIdx(mid_z);
    const VolumeStoreLayer &layer = m_volume_store.getLayer(layer_idx);
    const Polygons &polygons = layer.getPolygons();

    Wire wire;
    size_t num_polygons = polygons.size();
    for (size_t polygon_idx=0U; polygon_idx!=num_polygons; ++polygon_idx) {
        const PolygonRef polygon = polygons[polygon_idx];
        size_t num_points = polygon.size();
        assert(num_points > 0U);
        const Point *prev_point = &polygon[0];
        for (size_t point_idx=1U; point_idx<num_points; ++point_idx) {
            const Point *point = &polygon[point_idx];

            Point3 start_pt(prev_point->X,prev_point->Y,top_z);
            Point3 end_pt  (point     ->X,point     ->Y,top_z);
            wire.pt0 = start_pt;
            wire.pt1 = end_pt;
            wire.width = mmToInt(0.5);
            wire.height = height;
            m_wire_plan.addWire(wire);

            prev_point = point;
        }
    }
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

    m_volume_store.addMeshGroup(mesh_group);

    createWirePlan();
    createPrintPlan();
    
    m_plan_to_gcode.process(settings, &m_print_plan, &m_gcode_out);
}

}
