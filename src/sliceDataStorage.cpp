#include "sliceDataStorage.h"

#include "FffProcessor.h" //To create a mesh group with if none is provided.

namespace cura
{

Polygons SliceLayer::getOutlines(bool external_polys_only)
{
    Polygons ret;
    getOutlines(ret, external_polys_only);
    return ret;
}

void SliceLayer::getOutlines(Polygons& result, bool external_polys_only)
{
    for (SliceLayerPart& part : parts)
    {
        if (external_polys_only)
        {
            result.add(part.outline.outerPolygon());
        }
        else 
        {
            result.add(part.outline);
        }
    }
}

SliceDataStorage::SliceDataStorage(MeshGroup* meshgroup) : SettingsMessenger(meshgroup),
    meshgroup(meshgroup != nullptr ? meshgroup : new MeshGroup(FffProcessor::getInstance())), //If no mesh group is provided, we roll our own.
    retraction_config_per_extruder(initializeRetractionConfigs()),
    travel_config(&retraction_config, "MOVE"),
    skirt_config(initializeSkirtConfigs()),
    raft_base_config(&retraction_config_per_extruder[this->meshgroup->getSettingAsIndex("adhesion_extruder_nr")], "SUPPORT"),
    raft_interface_config(&retraction_config_per_extruder[this->meshgroup->getSettingAsIndex("adhesion_extruder_nr")], "SUPPORT"),
    raft_surface_config(&retraction_config_per_extruder[this->meshgroup->getSettingAsIndex("adhesion_extruder_nr")], "SUPPORT"),
    support_config(&retraction_config_per_extruder[this->meshgroup->getSettingAsIndex("support_extruder_nr")], "SUPPORT"),
    support_roof_config(&retraction_config_per_extruder[this->meshgroup->getSettingAsIndex("support_roof_extruder_nr")], "SKIN"),
    max_object_height_second_to_last_extruder(-1)
{
}

Polygons SliceDataStorage::getLayerOutlines(int layer_nr, bool include_helper_parts, bool external_polys_only)
{
    if (layer_nr < 0)
    { // when processing raft
        if (include_helper_parts)
        {
            if (external_polys_only)
            {
                std::vector<PolygonsPart> parts = raftOutline.splitIntoParts();
                Polygons result;
                for (PolygonsPart& part : parts) 
                {
                    result.add(part.outerPolygon());
                }
                return result;
            }
            else 
            {
                return raftOutline;
            }
        }
        else 
        {
            return Polygons();
        }
    }
    else 
    {
        Polygons total;
        for (SliceMeshStorage& mesh : meshes)
        {
            SliceLayer& layer = mesh.layers[layer_nr];
            layer.getOutlines(total, external_polys_only);
            if (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::NORMAL)
            {
                total = total.unionPolygons(layer.openPolyLines.offsetPolyLine(100));
            }
        }
        if (include_helper_parts)
        {
            if (support.generated) 
            {
                total.add(support.supportLayers[layer_nr].supportAreas);
                total.add(support.supportLayers[layer_nr].roofs);
            }
            total.add(primeTower.ground_poly);
        }
        return total;
    }
}



std::vector<bool> SliceDataStorage::getExtrudersUsed(int layer_nr)
{
    std::vector<bool> ret;
    ret.resize(meshgroup->getExtruderCount(), false);
    if (layer_nr < 0)
    {
        ret[getSettingAsIndex("adhesion_extruder_nr")] = true; // raft
    }
    else 
    {
        if (layer_nr == 0)
        { // process brim/skirt
            for (int extr_nr = 0; extr_nr < meshgroup->getExtruderCount(); extr_nr++)
            {
                if (skirt[extr_nr].size() > 0)
                {
                    ret[extr_nr] = true;
                    continue;
                }
            }
        }

        // TODO: ooze shield, draft shield

        // support
        if (support.supportLayers[layer_nr].supportAreas.size() > 0)
        {
            if (layer_nr == 0)
            {
                ret[getSettingAsIndex("support_extruder_nr_layer_0")] = true;
            }
            else 
            {
                ret[getSettingAsIndex("support_extruder_nr")] = true;
            }
        }
        if (support.supportLayers[layer_nr].roofs.size() > 0)
        {
            ret[getSettingAsIndex("support_roof_extruder_nr")] = true;
        }

        for (SliceMeshStorage& mesh : meshes)
        {
            SliceLayer& layer = mesh.layers[layer_nr];
            int extr_nr = mesh.getSettingAsIndex("extruder_nr");
            if (layer.parts.size() > 0)
            {
                ret[extr_nr] = true;
            }
        }
    }
    return ret;
}

std::vector< bool > SliceDataStorage::getExtrudersUsed()
{

    std::vector<bool> ret;
    ret.resize(meshgroup->getExtruderCount(), false);

    ret[getSettingAsIndex("adhesion_extruder_nr")] = true; 
    { // process brim/skirt
        for (int extr_nr = 0; extr_nr < meshgroup->getExtruderCount(); extr_nr++)
        {
            if (skirt[extr_nr].size() > 0)
            {
                ret[extr_nr] = true;
                continue;
            }
        }
    }

    // TODO: ooze shield, draft shield ..?

    // support
    // support is presupposed to be present...
    ret[getSettingAsIndex("support_extruder_nr_layer_0")] = true;
    ret[getSettingAsIndex("support_extruder_nr")] = true;
    ret[getSettingAsIndex("support_roof_extruder_nr")] = true;

    // all meshes are presupposed to actually have content
    for (SliceMeshStorage& mesh : meshes)
    {
        ret[mesh.getSettingAsIndex("extruder_nr")] = true;
    }
    return ret;
}



























} // namespace cura