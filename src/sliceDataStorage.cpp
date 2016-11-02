#include "sliceDataStorage.h"

#include "FffProcessor.h" //To create a mesh group with if none is provided.

namespace cura
{

Polygons& SliceLayerPart::getOwnInfillArea()
{
    if (infill_area_own)
    {
        return *infill_area_own;
    }
    else
    {
        return infill_area;
    }
}

Polygons SliceLayer::getOutlines(bool external_polys_only) const
{
    Polygons ret;
    getOutlines(ret, external_polys_only);
    return ret;
}

void SliceLayer::getOutlines(Polygons& result, bool external_polys_only) const
{
    for (const SliceLayerPart& part : parts)
    {
        if (external_polys_only)
        {
            result.add(const_cast<SliceLayerPart&>(part).outline.outerPolygon()); // TODO: make a const version of outerPolygon()
        }
        else 
        {
            result.add(part.print_outline);
        }
    }
}

Polygons SliceLayer::getSecondOrInnermostWalls() const
{
    Polygons ret;
    getSecondOrInnermostWalls(ret);
    return ret;
}

void SliceLayer::getSecondOrInnermostWalls(Polygons& layer_walls) const
{
    for (const SliceLayerPart& part : parts)
    {
        // we want the 2nd inner walls
        if (part.insets.size() >= 2) {
            layer_walls.add(const_cast<SliceLayerPart&>(part).insets[1]); // TODO const cast!
            continue;
        }
        // but we'll also take the inner wall if the 2nd doesn't exist
        if (part.insets.size() == 1) {
            layer_walls.add(const_cast<SliceLayerPart&>(part).insets[0]); // TODO const cast!
            continue;
        }
        // offset_from_outlines was so large that it completely destroyed our isle,
        // so we'll just use the regular outline
        layer_walls.add(part.outline);
        continue;
    }
}

std::vector<RetractionConfig> SliceDataStorage::initializeRetractionConfigs()
{
    std::vector<RetractionConfig> ret;
    ret.resize(meshgroup->getExtruderCount()); // initializes with constructor RetractionConfig()
    return ret;
}

std::vector<GCodePathConfig> SliceDataStorage::initializeTravelConfigs()
{
    std::vector<GCodePathConfig> ret;
    for (int extruder = 0; extruder < meshgroup->getExtruderCount(); extruder++)
    {
        travel_config_per_extruder.emplace_back(PrintFeatureType::MoveCombing);
    }
    return ret;
}

std::vector<GCodePathConfig> SliceDataStorage::initializeSkirtBrimConfigs()
{
    std::vector<GCodePathConfig> ret;
    for (int extruder = 0; extruder < meshgroup->getExtruderCount(); extruder++)
    {
        skirt_brim_config.emplace_back(PrintFeatureType::SkirtBrim);
    }
    return ret;
}

SliceDataStorage::SliceDataStorage(MeshGroup* meshgroup) : SettingsMessenger(meshgroup),
    meshgroup(meshgroup != nullptr ? meshgroup : new MeshGroup(FffProcessor::getInstance())), //If no mesh group is provided, we roll our own.
    retraction_config_per_extruder(initializeRetractionConfigs()),
    extruder_switch_retraction_config_per_extruder(initializeRetractionConfigs()),
    travel_config_per_extruder(initializeTravelConfigs()),
    skirt_brim_config(initializeSkirtBrimConfigs()),
    raft_base_config(PrintFeatureType::SupportInterface),
    raft_interface_config(PrintFeatureType::Support),
    raft_surface_config(PrintFeatureType::SupportInterface),
    support_config(PrintFeatureType::Support),
    support_skin_config(PrintFeatureType::SupportInterface),
    max_print_height_second_to_last_extruder(-1)
{
}

Polygons SliceDataStorage::getLayerOutlines(int layer_nr, bool include_helper_parts, bool external_polys_only) const
{
    if (layer_nr < 0 && layer_nr < -Raft::getFillerLayerCount(*this))
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
        if (layer_nr >= 0)
        {
            for (const SliceMeshStorage& mesh : meshes)
            {
                if (mesh.getSettingBoolean("infill_mesh"))
                {
                    continue;
                }
                const SliceLayer& layer = mesh.layers[layer_nr];
                layer.getOutlines(total, external_polys_only);
                if (const_cast<SliceMeshStorage&>(mesh).getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::NORMAL) // TODO: make all getSetting functions const??
                {
                    total = total.unionPolygons(layer.openPolyLines.offsetPolyLine(100));
                }
            }
        }
        if (include_helper_parts)
        {
            if (support.generated) 
            {
                total.add(support.supportLayers[std::max(0, layer_nr)].supportAreas);
                total.add(support.supportLayers[std::max(0, layer_nr)].skin);
            }
            total.add(primeTower.ground_poly);
        }
        return total;
    }
}

Polygons SliceDataStorage::getLayerSecondOrInnermostWalls(int layer_nr, bool include_helper_parts) const
{
    if (layer_nr < 0 && layer_nr < -Raft::getFillerLayerCount(*this))
    { // when processing raft
        if (include_helper_parts)
        {
            return raftOutline;
        }
        else 
        {
            return Polygons();
        }
    }
    else 
    {
        Polygons total;
        if (layer_nr >= 0)
        {
            for (const SliceMeshStorage& mesh : meshes)
            {
                const SliceLayer& layer = mesh.layers[layer_nr];
                layer.getSecondOrInnermostWalls(total);
                if (const_cast<SliceMeshStorage&>(mesh).getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::NORMAL) // TODO: make getSetting const? make settings.setting_values mapping mutable??
                {
                    total = total.unionPolygons(layer.openPolyLines.offsetPolyLine(100));
                }
            }
        }
        if (include_helper_parts)
        {
            if (support.generated) 
            {
                total.add(support.supportLayers[std::max(0, layer_nr)].supportAreas);
                total.add(support.supportLayers[std::max(0, layer_nr)].skin);
            }
            total.add(primeTower.ground_poly);
        }
        return total;
    }

}

std::vector<bool> SliceDataStorage::getExtrudersUsed() const
{

    std::vector<bool> ret;
    ret.resize(meshgroup->getExtruderCount(), false);

    ret[getSettingAsIndex("adhesion_extruder_nr")] = true; 
    { // process brim/skirt
        for (int extr_nr = 0; extr_nr < meshgroup->getExtruderCount(); extr_nr++)
        {
            if (skirt_brim[extr_nr].size() > 0)
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
    ret[getSettingAsIndex("support_infill_extruder_nr")] = true;
    ret[getSettingAsIndex("support_interface_extruder_nr")] = true;

    // all meshes are presupposed to actually have content
    for (const SliceMeshStorage& mesh : meshes)
    {
        ret[mesh.getSettingAsIndex("extruder_nr")] = true;
    }
    return ret;
}

std::vector<bool> SliceDataStorage::getExtrudersUsed(int layer_nr) const
{

    std::vector<bool> ret;
    ret.resize(meshgroup->getExtruderCount(), false);

    bool include_adhesion = true;
    bool include_helper_parts = true;
    bool include_models = true;
    if (layer_nr < 0)
    {
        include_models = false;
        if (layer_nr < -Raft::getFillerLayerCount(*this))
        {
            include_helper_parts = false;
        }
        else
        {
            layer_nr = 0; // because the helper parts are copied from the initial layer in the filler layer
            include_adhesion = false;
        }
    }
    if (include_adhesion)
    {
        ret[getSettingAsIndex("adhesion_extruder_nr")] = true;
        { // process brim/skirt
            for (int extr_nr = 0; extr_nr < meshgroup->getExtruderCount(); extr_nr++)
            {
                if (skirt_brim[extr_nr].size() > 0)
                {
                    ret[extr_nr] = true;
                    continue;
                }
            }
        }
    }

    // TODO: ooze shield, draft shield ..?

    if (include_helper_parts)
    {
        // support
        if (layer_nr < int(support.supportLayers.size()))
        {
            const SupportLayer& support_layer = support.supportLayers[layer_nr];
            if (layer_nr == 0)
            {
                if (support_layer.supportAreas.size() > 0)
                {
                    ret[getSettingAsIndex("support_extruder_nr_layer_0")] = true;
                }
            }
            else
            {
                if (support_layer.supportAreas.size() > 0)
                {
                    ret[getSettingAsIndex("support_infill_extruder_nr")] = true;
                }
            }
            if (support_layer.skin.size() > 0)
            {
                ret[getSettingAsIndex("support_interface_extruder_nr")] = true;
            }
        }
    }

    if (include_models)
    {
        for (const SliceMeshStorage& mesh : meshes)
        {
            if (layer_nr >= int(mesh.layers.size()))
            {
                continue;
            }
            const SliceLayer& layer = mesh.layers[layer_nr];
            if (layer.parts.size() > 0)
            {
                ret[mesh.getSettingAsIndex("extruder_nr")] = true;
            }
        }
    }
    return ret;
}



























} // namespace cura