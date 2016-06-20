/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#include "MasonBackend.hpp"

namespace cura {
namespace mason {

MasonBackend::MasonBackend()
{
}

void MasonBackend::createPlan()
{
    std::shared_ptr<PrintPlanElement> elem;

    Point3 start_pt(MM2INT(150.0),MM2INT(140.0),MM2INT(0.25));
    elem.reset(new HeadMove(start_pt,30.0));
    m_print_plan.addElement(elem);
    Point3 end_pt(MM2INT(170.0),MM2INT(140.0),MM2INT(0.25));
    elem.reset(new ExtrudedSegment(start_pt,end_pt,30.0,0.5*0.25));
    m_print_plan.addElement(elem);
}

void MasonBackend::process(const SettingsBaseVirtual *settings, const MeshGroup *mesh_group, GCodeExport *gcode_out)
{
    gcode_out->preSetup(mesh_group);

    m_volume_store.addMeshGroup(mesh_group);

    createPlan();
    
    m_plan_to_gcode.process(settings, &m_print_plan, gcode_out);
}

}
}
