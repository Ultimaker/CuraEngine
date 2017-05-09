//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "sliceDataStorage.h"

#include "FffProcessor.h" //To create a mesh group with if none is provided.
#include "infill/SubDivCube.h" // For the destructor


namespace cura
{

Polygons& SliceLayerPart::getOwnInfillArea()
{
    return const_cast<Polygons&>(const_cast<const SliceLayerPart*>(this)->getOwnInfillArea());
}

const Polygons& SliceLayerPart::getOwnInfillArea() const
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
            result.add(part.outline.outerPolygon());
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
            layer_walls.add(part.insets[1]);
            continue;
        }
        // but we'll also take the inner wall if the 2nd doesn't exist
        if (part.insets.size() == 1) {
            layer_walls.add(part.insets[0]);
            continue;
        }
        // offset_from_outlines was so large that it completely destroyed our isle,
        // so we'll just use the regular outline
        layer_walls.add(part.outline);
        continue;
    }
}

SliceMeshStorage::~SliceMeshStorage()
{
    if (base_subdiv_cube)
    {
        delete base_subdiv_cube;
    }
}

bool SliceMeshStorage::getExtruderIsUsed(int extruder_nr) const
{
    if (getSettingBoolean("magic_spiralize"))
    {
        if (getSettingAsIndex("wall_0_extruder_nr") == extruder_nr)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    if (getSettingAsCount("wall_line_count") > 0 && getSettingAsIndex("wall_0_extruder_nr") == extruder_nr)
    {
        return true;
    }
    if ((getSettingAsCount("wall_line_count") > 1 || getSettingBoolean("alternate_extra_perimeter") || getSettingBoolean("fill_perimeter_gaps"))
        && getSettingAsIndex("wall_x_extruder_nr") == extruder_nr)
    {
        return true;
    }
    if (getSettingInMicrons("infill_line_distance") > 0 && getSettingAsIndex("infill_extruder_nr") == extruder_nr)
    {
        return true;
    }
    if ((getSettingAsCount("top_layers") > 0 || getSettingAsCount("bottom_layers")) && getSettingAsIndex("top_bottom_extruder_nr") == extruder_nr)
    {
        return true;
    }
    return false;
}

bool SliceMeshStorage::getExtruderIsUsed(int extruder_nr, int layer_nr) const
{
    if (layer_nr < 0 || layer_nr >= static_cast<int>(layers.size()))
    {
        return false;
    }
    if (getSettingBoolean("anti_overhang_mesh")
        || getSettingBoolean("support_mesh"))
    { // object is not printed as object, but as support.
        return false;
    }
    const SliceLayer& layer = layers[layer_nr];
    if (getSettingAsCount("wall_line_count") > 0 && getSettingAsIndex("wall_0_extruder_nr") == extruder_nr)
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            if (part.insets.size() > 0 && part.insets[0].size() > 0)
            {
                return true;
            }
        }
    }
    if (getSettingAsFillPerimeterGapMode("fill_perimeter_gaps") != FillPerimeterGapMode::NOWHERE
        && (getSettingAsCount("wall_line_count") > 0 || getSettingAsCount("skin_outline_count") > 0)
        && getSettingAsIndex("wall_0_extruder_nr") == extruder_nr)
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            if (part.perimeter_gaps.size() > 0)
            {
                return true;
            }
            for (const SkinPart& skin_part : part.skin_parts)
            {
                if (skin_part.perimeter_gaps.size() > 0)
                {
                    return true;
                }
            }
        }
    }
    if ((getSettingAsCount("wall_line_count") > 1 || getSettingBoolean("alternate_extra_perimeter")) && getSettingAsIndex("wall_x_extruder_nr") == extruder_nr)
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            if (part.insets.size() > 1 && part.insets[1].size() > 0)
            {
                return true;
            }
        }
    }
    if (getSettingInMicrons("infill_line_distance") > 0 && getSettingAsIndex("infill_extruder_nr") == extruder_nr)
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            if (part.getOwnInfillArea().size() > 0)
            {
                return true;
            }
        }
    }
    if (getSettingAsIndex("top_bottom_extruder_nr") == extruder_nr)
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            if (!part.skin_parts.empty())
            {
                return true;
            }
        }
    }
    return false;
}

std::vector<RetractionConfig> SliceDataStorage::initializeRetractionConfigs()
{
    std::vector<RetractionConfig> ret;
    ret.resize(meshgroup->getExtruderCount()); // initializes with constructor RetractionConfig()
    return ret;
}

SliceDataStorage::SliceDataStorage(MeshGroup* meshgroup) : SettingsMessenger(meshgroup),
    meshgroup(meshgroup != nullptr ? meshgroup : new MeshGroup(FffProcessor::getInstance())), //If no mesh group is provided, we roll our own.
    print_layer_count(0),
    retraction_config_per_extruder(initializeRetractionConfigs()),
    extruder_switch_retraction_config_per_extruder(initializeRetractionConfigs()),
    max_print_height_second_to_last_extruder(-1),
    primeTower(*this)
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
                if (mesh.getSettingBoolean("infill_mesh") || mesh.getSettingBoolean("anti_overhang_mesh"))
                {
                    continue;
                }
                const SliceLayer& layer = mesh.layers[layer_nr];
                layer.getOutlines(total, external_polys_only);
                if (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::NORMAL)
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
                total.add(support.supportLayers[std::max(0, layer_nr)].support_bottom);
                total.add(support.supportLayers[std::max(0, layer_nr)].support_roof);
            }
            if (primeTower.enabled)
            {
                total.add(primeTower.ground_poly);
            }
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
                if (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::NORMAL)
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
                total.add(support.supportLayers[std::max(0, layer_nr)].support_bottom);
                total.add(support.supportLayers[std::max(0, layer_nr)].support_roof);
            }
            if (primeTower.enabled)
            {
                total.add(primeTower.ground_poly);
            }
        }
        return total;
    }

}

std::vector<bool> SliceDataStorage::getExtrudersUsed() const
{

    std::vector<bool> ret;
    ret.resize(meshgroup->getExtruderCount(), false);

    if (getSettingAsPlatformAdhesion("adhesion_type") != EPlatformAdhesion::NONE)
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

    // support
    // support is presupposed to be present...
    for (const SliceMeshStorage& mesh : meshes)
    {
        if (mesh.getSettingBoolean("support_enable") || mesh.getSettingBoolean("support_mesh"))
        {
            ret[getSettingAsIndex("support_extruder_nr_layer_0")] = true;
            ret[getSettingAsIndex("support_infill_extruder_nr")] = true;
            if (getSettingBoolean("support_roof_enable"))
            {
                ret[getSettingAsIndex("support_roof_extruder_nr")] = true;
            }
            if (getSettingBoolean("support_bottom_enable"))
            {
                ret[getSettingAsIndex("support_bottom_extruder_nr")] = true;
            }
        }
    }

    // all meshes are presupposed to actually have content
    for (const SliceMeshStorage& mesh : meshes)
    {
        for (unsigned int extruder_nr = 0; extruder_nr <= ret.size(); extruder_nr++)
        {
            ret[extruder_nr] = ret[extruder_nr] || mesh.getExtruderIsUsed(extruder_nr);
        }
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
    else if (layer_nr > 0 || getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT)
    { // only include adhesion only for layers where platform adhesion actually occurs
        // i.e. layers < 0 are for raft, layer 0 is for brim/skirt
        include_adhesion = false;
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
                if (!support_layer.supportAreas.empty())
                {
                    ret[getSettingAsIndex("support_extruder_nr_layer_0")] = true;
                }
            }
            else
            {
                if (!support_layer.supportAreas.empty())
                {
                    ret[getSettingAsIndex("support_infill_extruder_nr")] = true;
                }
            }
            if (!support_layer.support_bottom.empty())
            {
                ret[getSettingAsIndex("support_bottom_extruder_nr")] = true;
            }
            if (!support_layer.support_roof.empty())
            {
                ret[getSettingAsIndex("support_roof_extruder_nr")] = true;
            }
        }
    }

    if (include_models)
    {
        for (const SliceMeshStorage& mesh : meshes)
        {
            for (unsigned int extruder_nr = 0; extruder_nr <= ret.size(); extruder_nr++)
            {
                ret[extruder_nr] = ret[extruder_nr] || mesh.getExtruderIsUsed(extruder_nr, layer_nr);
            }
        }
    }
    return ret;
}



























} // namespace cura