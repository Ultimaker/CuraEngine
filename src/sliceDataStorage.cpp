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
    travel_config(&retraction_config, "MOVE"),
    raft_base_config(&retraction_config_per_extruder[meshgroup->getSettingAsIndex("adhesion_extruder_nr")], "SUPPORT"),
    raft_interface_config(&retraction_config_per_extruder[meshgroup->getSettingAsIndex("adhesion_extruder_nr")], "SUPPORT"),
    raft_surface_config(&retraction_config_per_extruder[meshgroup->getSettingAsIndex("adhesion_extruder_nr")], "SUPPORT"),
    support_config(&retraction_config_per_extruder[meshgroup->getSettingAsIndex("support_extruder_nr")], "SUPPORT"),
    support_roof_config(&retraction_config_per_extruder[meshgroup->getSettingAsIndex("support_roof_extruder_nr")], "SKIN")
{
    this->meshgroup = meshgroup;
    if(!(this->meshgroup))
    {
        this->meshgroup = new MeshGroup(FffProcessor::getInstance());
    }
    retraction_config_per_extruder = initializeRetractionConfigs();
    skirt_config = initializeSkirtConfigs();
    max_object_height_second_to_last_extruder = -1;
    ;
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






























} // namespace cura