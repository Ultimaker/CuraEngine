//Copyright (c) 2016 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "sliceDataStorage.h"

#include "FffProcessor.h" //To create a mesh group with if none is provided.
#include "infill/SubDivCube.h" // For the destructor


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

void SliceMeshStorage::findLayerSeamsForSpiralize()
{
    // The spiral has to continue on in an anti-clockwise direction from where the last layer finished, it can't jump backwards

    layer_seam_vertex_indices.clear();
    layer_seam_vertex_indices.push_back(0); // got to start somewhere

    for (unsigned layer_nr = 1; layer_nr < layers.size(); ++layer_nr)
    {
        const SliceLayer& layer = layers[layer_nr];
        const SliceLayer& last_layer = layers[layer_nr - 1];

        if (layer.parts.size() < 1 || layer.parts[0].insets.size() < 1 || last_layer.parts.size() < 1 || last_layer.parts[0].insets.size() < 1)
        {
            // either this layer or the last has no parts (or a part has no insets) so just duplicate the index calculated for the last layer
            layer_seam_vertex_indices.push_back(layer_seam_vertex_indices[layer_nr - 1]);
            continue;
        }
        ConstPolygonRef last_wall = last_layer.parts[0].insets[0][0];
        ConstPolygonRef wall = layer.parts[0].insets[0][0];
        const int n_points = wall.size();
        Point last_wall_seam_vertex = last_wall[layer_seam_vertex_indices[layer_nr - 1]];

        // seam_vertex_idx is going to be the index of the seam vertex in the current wall polygon
        // initially we choose the vertex that is closest to the seam vertex in the last spiralized layer processed

        int seam_vertex_idx = PolygonUtils::findClosest(last_wall_seam_vertex, wall).point_idx;

        // now we check that the vertex following the seam vertex is not to the right of the seam vertex in the last layer
        // and if it is we move forward

        // get the inward normal of the last layer seam vertex
        Point last_wall_seam_vertex_inward_normal = PolygonUtils::getVertexInwardNormal(last_wall, layer_seam_vertex_indices[layer_nr - 1]);

        // create a vector from the normal so that we can then test the vertex following the candidate seam vertex to make sure it is on the correct side
        Point last_wall_seam_vertex_vector = last_wall_seam_vertex + last_wall_seam_vertex_inward_normal;

        // now test the vertex following the candidate seam vertex and if it lies to the left or on the vector, it's good to use
        const int first_seam_vertex_idx = seam_vertex_idx;
        float a = LinearAlg2D::getAngleLeft(last_wall_seam_vertex_vector, last_wall_seam_vertex, wall[(seam_vertex_idx + 1) % n_points]);
        //std::cerr << layer_nr << ": n_points= " << n_points << ", angle = " << a * 180/M_PI << "\n";

        while (a > M_PI)
        {
            // the vertex was on the right of the vector so move the seam vertex on
            seam_vertex_idx = (seam_vertex_idx + 1) % n_points;
            a = LinearAlg2D::getAngleLeft(last_wall_seam_vertex_vector, last_wall_seam_vertex, wall[(seam_vertex_idx + 1) % n_points]);
            //std::cerr << "new angle = " << a * 180/M_PI << "\n";

            if (seam_vertex_idx == first_seam_vertex_idx)
            {
                // this shouldn't happen!
                break;
            }
        }

        // store result for use when spiralizing
        layer_seam_vertex_indices.push_back(seam_vertex_idx);
        //std::cerr << "layer " << layer_nr << ": " << seam_vertex_idx << "\n";
    }
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
                total.add(support.supportLayers[std::max(0, layer_nr)].skin);
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
                total.add(support.supportLayers[std::max(0, layer_nr)].skin);
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
    ret[getSettingAsIndex("support_extruder_nr_layer_0")] = true;
    ret[getSettingAsIndex("support_infill_extruder_nr")] = true;
    ret[getSettingAsIndex("support_interface_extruder_nr")] = true;

    // all meshes are presupposed to actually have content
    for (const SliceMeshStorage& mesh : meshes)
    {
        if (!mesh.getSettingBoolean("anti_overhang_mesh")
            && !mesh.getSettingBoolean("support_mesh")
        )
        {
            ret[mesh.getSettingAsIndex("extruder_nr")] = true;
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