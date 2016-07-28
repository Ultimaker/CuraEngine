#include "FffProcessor.h" 

namespace cura 
{

FffProcessor FffProcessor::instance; // definition must be in cpp

FffProcessor::FffProcessor()
: polygon_generator(this)
, gcode_writer(this)
, meshgroup_number(0)
{
}

int FffProcessor::getMeshgroupNr()
{
    return meshgroup_number;
}


std::string FffProcessor::getAllSettingsString(MeshGroup& meshgroup, bool first_meshgroup)
{
    std::stringstream sstream;
    if (first_meshgroup)
    {
        sstream << getAllLocalSettingsString(); // global settings
        sstream << " -g";
    }
    else 
    {
        sstream << " --next";
    }
    sstream << meshgroup.getAllLocalSettingsString();
    for (int extruder_nr = 0; extruder_nr < meshgroup.getExtruderCount(); extruder_nr++)
    {
        ExtruderTrain* train = meshgroup.getExtruderTrain(extruder_nr);
        sstream << " -e" << extruder_nr << train->getAllLocalSettingsString();
    }
    for (unsigned int mesh_idx = 0; mesh_idx < meshgroup.meshes.size(); mesh_idx++)
    {
        Mesh& mesh = meshgroup.meshes[mesh_idx];
        sstream << " -e" << mesh.getSettingAsIndex("extruder_nr") << " -l \"" << mesh_idx << "\"" << mesh.getAllLocalSettingsString();
    }
    sstream << "\n";
    return sstream.str();
}

bool FffProcessor::processFiles(const std::vector< std::string >& files)
{
    time_keeper.restart();
    MeshGroup* meshgroup = new MeshGroup(this);
    
    for(std::string filename : files)
    {
        log("Loading %s from disk...\n", filename.c_str());

        FMatrix3x3 matrix;
        if (!loadMeshIntoMeshGroup(meshgroup, filename.c_str(), matrix))
        {
            logError("Failed to load model: %s\n", filename.c_str());
            return false;
        }
    }
    
    meshgroup->finalize();

    log("Loaded from disk in %5.3fs\n", time_keeper.restart());
    return processMeshGroup(meshgroup);
}

bool FffProcessor::processMeshGroup(MeshGroup* meshgroup)
{
    if (SHOW_ALL_SETTINGS) { logWarning(getAllSettingsString(*meshgroup, meshgroup_number == 0).c_str()); }
    time_keeper.restart();
    if (!meshgroup)
        return false;

    TimeKeeper time_keeper_total;

    polygon_generator.setParent(meshgroup);
    gcode_writer.setParent(meshgroup);

    bool empty = true;
    for (Mesh& mesh : meshgroup->meshes)
    {
        if (!mesh.getSettingBoolean("infill_mesh"))
        {
            empty = false;
        }
    }
    if (empty)
    {
        Progress::messageProgress(Progress::Stage::FINISH, 1, 1); // 100% on this meshgroup
        log("Total time elapsed %5.2fs.\n", time_keeper_total.restart());

        profile_string += getAllSettingsString(*meshgroup, meshgroup_number == 0);
        return true;
    }
    
    if (meshgroup->getSettingBoolean("wireframe_enabled"))
    {
        log("starting Neith Weaver...\n");
                    
        Weaver w(this);
        w.weave(meshgroup);
        
        log("starting Neith Gcode generation...\n");
        Wireframe2gcode gcoder(w, gcode_writer.gcode, this);
        gcoder.writeGCode();
        log("finished Neith Gcode generation...\n");
        
    } else 
    {
        SliceDataStorage storage(meshgroup);

        if (!polygon_generator.generateAreas(storage, meshgroup, time_keeper))
        {
            return false;
        }
        
        Progress::messageProgressStage(Progress::Stage::EXPORT, &time_keeper);
        gcode_writer.writeGCode(storage, time_keeper);
    }

    Progress::messageProgress(Progress::Stage::FINISH, 1, 1); // 100% on this meshgroup
    if (CommandSocket::isInstantiated())
    {
        CommandSocket::getInstance()->flushGcode();
        CommandSocket::getInstance()->sendOptimizedLayerData();
    }
    log("Total time elapsed %5.2fs.\n", time_keeper_total.restart());

    profile_string += getAllSettingsString(*meshgroup, meshgroup_number == 0);
    meshgroup_number++;

    polygon_generator.setParent(this); // otherwise consequent getSetting calls (e.g. for finalize) will refer to non-existent meshgroup
    gcode_writer.setParent(this); // otherwise consequent getSetting calls (e.g. for finalize) will refer to non-existent meshgroup

    return true;
}

} // namespace cura 