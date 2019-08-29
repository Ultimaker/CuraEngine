//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "Application.h" //To get settings.
#include "ExtruderTrain.h"
#include "FffProcessor.h" //To create a mesh group with if none is provided.
#include "raft.h"
#include "Slice.h"
#include "sliceDataStorage.h"
#include "infill/SierpinskiFillProvider.h"
#include "infill/SubDivCube.h" // For the destructor
#include "infill/DensityProvider.h" // for destructor
#include "utils/math.h" //For PI.
#include "utils/logoutput.h"


namespace cura
{

SupportStorage::SupportStorage()
: generated(false)
, layer_nr_max_filled_layer(-1)
, cross_fill_provider(nullptr)
{
}

SupportStorage::~SupportStorage()
{
    supportLayers.clear(); 
    if (cross_fill_provider)
    {
        delete cross_fill_provider;
    }
}

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

SliceLayer::~SliceLayer()
{
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

Polygons& SliceLayer::getInnermostWalls(const size_t max_inset, const SliceMeshStorage& mesh) const
{
    if (innermost_walls_cache.count(max_inset) > 0)
    {
        return innermost_walls_cache[max_inset];
    }
    Polygons& result = innermost_walls_cache.emplace(std::make_pair(max_inset, Polygons())).first->second;

    const coord_t half_line_width_0 = mesh.settings.get<coord_t>("wall_line_width_0") / 2;
    const coord_t half_line_width_x = mesh.settings.get<coord_t>("wall_line_width_x") / 2;

    for (const SliceLayerPart& part : parts)
    {
        Polygons outer; // outer boundary limit (centre of 1st wall where present, otherwise part's outline)

        if (part.insets.size() > 0)
        {
            // part has at least one wall, test if it is complete

            if (part.insets[0].size() == part.outline.size())
            {
                // 1st wall is complete, use it for the outer boundary
                outer = part.insets[0];
            }
            else
            {
                // 1st wall is incomplete, merge the 1st wall with the part's outline (where the 1st wall is missing)

                // first we calculate the part outline for those portions of the part where the 1st wall is missing
                // this is done by shrinking the part outline so that it is very slightly smaller than the 1st wall outline, then expanding it again so it is very
                // slightly larger than its original size and subtracting that from the original part outline
                // NOTE - the additional small shrink/expands are required to ensure that the polygons overlap a little so we do not rely on exact results

                Polygons outline_where_there_are_no_inner_insets(part.outline.difference(part.outline.offset(-(half_line_width_0+5)).offset(half_line_width_0+10)));

                // merge the 1st wall outline with the portions of the part outline we just calculated
                // the trick here is to expand the outlines sufficiently so that they overlap when unioned and then the result is shrunk back to the correct size

                outer = part.insets[0].offset(half_line_width_0).unionPolygons(outline_where_there_are_no_inner_insets.offset(half_line_width_0)).offset(-half_line_width_0);
            }
        }
        else
        {
            // part has no walls, just use its outline
            outer = part.outline;
        }

        if (max_inset >= 2 && part.insets.size() >= 2)
        {
            // use the 2nd wall - if the 2nd wall is incomplete because the part is narrow, we use the 2nd wall where it does exist
            // and where it is missing, we use outer instead

            const coord_t inset_spacing = half_line_width_0 + half_line_width_x; // distance between the centre lines of the 1st and 2nd walls

            // first we calculate the regions of outer that correspond to where the 2nd wall is missing using a similar technique to what we used to calculate outer

            Polygons outer_where_there_are_no_inner_insets(outer.difference(outer.offset(-(inset_spacing+5)).offset(inset_spacing+10)));

            if (outer_where_there_are_no_inner_insets.size() > 0)
            {
                // there are some regions where the 2nd wall is missing so we must merge the 2nd wall outline
                // with the portions of outer we just calculated

                result.add(part.insets[1].offset(half_line_width_x).unionPolygons(outer_where_there_are_no_inner_insets.offset(half_line_width_0 + 15)).offset(-std::min(half_line_width_0, half_line_width_x)));
            }
            else
            {
                // the 2nd wall is complete so use it verbatim
                result.add(part.insets[1]);
            }
        }
        else
        {
            // fall back to using outer computed above
            result.add(outer);
        }
    }

    return result;
}

SliceMeshStorage::SliceMeshStorage(Mesh* mesh, const size_t slice_layer_count)
: settings(mesh->settings)
, mesh_name(mesh->mesh_name)
, layer_nr_max_filled_layer(0)
, bounding_box(mesh->getAABB())
, base_subdiv_cube(nullptr)
, cross_fill_provider(nullptr)
{
    layers.resize(slice_layer_count);
}

SliceMeshStorage::~SliceMeshStorage()
{
    if (base_subdiv_cube)
    {
        delete base_subdiv_cube;
    }
    if (cross_fill_provider)
    {
        delete cross_fill_provider;
    }
}

bool SliceMeshStorage::getExtruderIsUsed(const size_t extruder_nr) const
{
    if (settings.get<bool>("anti_overhang_mesh")
        || settings.get<bool>("support_mesh"))
    { // object is not printed as object, but as support.
        return false;
    }
    if (settings.get<bool>("magic_spiralize"))
    {
        if (settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr == extruder_nr)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    if (settings.get<size_t>("wall_line_count") > 0 && settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr == extruder_nr)
    {
        return true;
    }
    if ((settings.get<size_t>("wall_line_count") > 1 || settings.get<bool>("alternate_extra_perimeter") || settings.get<bool>("fill_perimeter_gaps"))
        && settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr == extruder_nr)
    {
        return true;
    }
    if (settings.get<coord_t>("infill_line_distance") > 0 && settings.get<ExtruderTrain&>("infill_extruder_nr").extruder_nr == extruder_nr)
    {
        return true;
    }
    if ((settings.get<size_t>("top_layers") > 0 || settings.get<size_t>("bottom_layers") > 0) && settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr == extruder_nr)
    {
        return true;
    }
    if ((settings.get<size_t>("top_layers") > 0 || settings.get<size_t>("bottom_layers") > 0) && settings.get<size_t>("roofing_layer_count") > 0 && settings.get<ExtruderTrain&>("roofing_extruder_nr").extruder_nr == extruder_nr)
    {
        return true;
    }
    return false;
}

bool SliceMeshStorage::getExtruderIsUsed(const size_t extruder_nr, const LayerIndex& layer_nr) const
{
    if (layer_nr < 0 || layer_nr >= static_cast<int>(layers.size()))
    {
        return false;
    }
    if (settings.get<bool>("anti_overhang_mesh")
        || settings.get<bool>("support_mesh"))
    { // object is not printed as object, but as support.
        return false;
    }
    const SliceLayer& layer = layers[layer_nr];
    if (settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr == extruder_nr && (settings.get<size_t>("wall_line_count") > 0 || settings.get<size_t>("skin_outline_count") > 0))
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            if (part.insets.size() > 0 && part.insets[0].size() > 0)
            {
                return true;
            }
            for (const SkinPart& skin_part : part.skin_parts)
            {
                if (!skin_part.insets.empty())
                {
                    return true;
                }
            }
        }
    }
    if (settings.get<FillPerimeterGapMode>("fill_perimeter_gaps") != FillPerimeterGapMode::NOWHERE
        && (settings.get<size_t>("wall_line_count") > 0 || settings.get<size_t>("skin_outline_count") > 0)
        && settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr == extruder_nr)
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
    if (settings.get<bool>("fill_outline_gaps")
        && settings.get<size_t>("wall_line_count") > 0
        && settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr == extruder_nr)
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            if (part.outline_gaps.size() > 0)
            {
                return true;
            }
        }
    }
    if ((settings.get<size_t>("wall_line_count") > 1 || settings.get<bool>("alternate_extra_perimeter")) && settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr == extruder_nr)
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            if (part.insets.size() > 1 && part.insets[1].size() > 0)
            {
                return true;
            }
        }
    }
    if (settings.get<coord_t>("infill_line_distance") > 0 && settings.get<ExtruderTrain&>("infill_extruder_nr").extruder_nr == extruder_nr)
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            if (part.getOwnInfillArea().size() > 0)
            {
                return true;
            }
        }
    }
    if (settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr == extruder_nr)
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            for (const SkinPart& skin_part : part.skin_parts)
            {
                if (!skin_part.inner_infill.empty())
                {
                    return true;
                }
            }
        }
    }
    if (settings.get<ExtruderTrain&>("roofing_extruder_nr").extruder_nr == extruder_nr)
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            for (const SkinPart& skin_part : part.skin_parts)
            {
                if (!skin_part.roofing_fill.empty())
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool SliceMeshStorage::isPrinted() const
{
    return !settings.get<bool>("infill_mesh") && !settings.get<bool>("cutting_mesh") && !settings.get<bool>("anti_overhang_mesh");
}

Point SliceMeshStorage::getZSeamHint() const
{
    Point pos(settings.get<coord_t>("z_seam_x"), settings.get<coord_t>("z_seam_y"));
    if (settings.get<bool>("z_seam_relative"))
    {
        Point3 middle = bounding_box.getMiddle();
        pos += Point(middle.x, middle.y);
    }
    return pos;
}

std::vector<RetractionConfig> SliceDataStorage::initializeRetractionConfigs()
{
    std::vector<RetractionConfig> ret;
    ret.resize(Application::getInstance().current_slice->scene.extruders.size()); // initializes with constructor RetractionConfig()
    return ret;
}

std::vector<WipeScriptConfig> SliceDataStorage::initializeWipeConfigs()
{
    std::vector<WipeScriptConfig> ret;
    ret.resize(Application::getInstance().current_slice->scene.extruders.size());
    return ret;
}

SliceDataStorage::SliceDataStorage()
: print_layer_count(0)
, wipe_config_per_extruder(initializeWipeConfigs())
, retraction_config_per_extruder(initializeRetractionConfigs())
, extruder_switch_retraction_config_per_extruder(initializeRetractionConfigs())
, max_print_height_second_to_last_extruder(-1)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    Point3 machine_max(mesh_group_settings.get<coord_t>("machine_width"), mesh_group_settings.get<coord_t>("machine_depth"), mesh_group_settings.get<coord_t>("machine_height"));
    Point3 machine_min(0, 0, 0);
    if (mesh_group_settings.get<bool>("machine_center_is_zero"))
    {
        machine_max /= 2;
        machine_min -= machine_max;
    }
    machine_size.include(machine_min);
    machine_size.include(machine_max);
}

Polygons SliceDataStorage::getLayerOutlines(const LayerIndex layer_nr, const bool include_support, const bool include_prime_tower, const bool external_polys_only) const
{
    if (layer_nr < 0 && layer_nr < -static_cast<LayerIndex>(Raft::getFillerLayerCount()))
    { // when processing raft
        if (include_support)
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
        coord_t maximum_resolution = std::numeric_limits<coord_t>::max();
        coord_t maximum_deviation = std::numeric_limits<coord_t>::max();
        if (layer_nr >= 0)
        {
            for (const SliceMeshStorage& mesh : meshes)
            {
                if (mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("anti_overhang_mesh"))
                {
                    continue;
                }
                const SliceLayer& layer = mesh.layers[layer_nr];
                layer.getOutlines(total, external_polys_only);
                if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL)
                {
                    total = total.unionPolygons(layer.openPolyLines.offsetPolyLine(100));
                }
                maximum_resolution = std::min(maximum_resolution, mesh.settings.get<coord_t>("meshfix_maximum_resolution"));
                maximum_deviation = std::min(maximum_deviation, mesh.settings.get<coord_t>("meshfix_maximum_deviation"));
            }
        }
        if (include_support)
        {
            const SupportLayer& support_layer = support.supportLayers[std::max(LayerIndex(0), layer_nr)];
            if (support.generated) 
            {
                for (const SupportInfillPart& support_infill_part : support_layer.support_infill_parts)
                {
                    total.add(support_infill_part.outline);
                }
                total.add(support_layer.support_bottom);
                total.add(support_layer.support_roof);
            }
        }
        if (include_prime_tower)
        {
            if (primeTower.enabled)
            {
                total.add(layer_nr == 0 ? primeTower.outer_poly_first_layer : primeTower.outer_poly);
            }
        }
        total.simplify(maximum_resolution, maximum_deviation);
        return total;
    }
}

std::vector<bool> SliceDataStorage::getExtrudersUsed() const
{
    std::vector<bool> ret;
    ret.resize(Application::getInstance().current_slice->scene.extruders.size(), false);

    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    if (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::NONE)
    {
        ret[mesh_group_settings.get<ExtruderTrain&>("adhesion_extruder_nr").extruder_nr] = true;
        { // process brim/skirt
            for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice->scene.extruders.size(); extruder_nr++)
            {
                if (skirt_brim[extruder_nr].size() > 0)
                {
                    ret[extruder_nr] = true;
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
        if (mesh.settings.get<bool>("support_enable") || mesh.settings.get<bool>("support_tree_enable") || mesh.settings.get<bool>("support_mesh"))
        {
            ret[mesh_group_settings.get<ExtruderTrain&>("support_extruder_nr_layer_0").extruder_nr] = true;
            ret[mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr] = true;
            if (mesh_group_settings.get<bool>("support_roof_enable"))
            {
                ret[mesh_group_settings.get<ExtruderTrain&>("support_roof_extruder_nr").extruder_nr] = true;
            }
            if (mesh_group_settings.get<bool>("support_bottom_enable"))
            {
                ret[mesh_group_settings.get<ExtruderTrain&>("support_bottom_extruder_nr").extruder_nr] = true;
            }
        }
    }

    // all meshes are presupposed to actually have content
    for (const SliceMeshStorage& mesh : meshes)
    {
        for (unsigned int extruder_nr = 0; extruder_nr < ret.size(); extruder_nr++)
        {
            ret[extruder_nr] = ret[extruder_nr] || mesh.getExtruderIsUsed(extruder_nr);
        }
    }
    return ret;
}

std::vector<bool> SliceDataStorage::getExtrudersUsed(LayerIndex layer_nr) const
{
    std::vector<bool> ret;
    ret.resize(Application::getInstance().current_slice->scene.extruders.size(), false);
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;

    bool include_adhesion = true;
    bool include_helper_parts = true;
    bool include_models = true;
    if (layer_nr < 0)
    {
        include_models = false;
        if (layer_nr < -static_cast<LayerIndex>(Raft::getFillerLayerCount()))
        {
            include_helper_parts = false;
        }
        else
        {
            layer_nr = 0; // because the helper parts are copied from the initial layer in the filler layer
            include_adhesion = false;
        }
    }
    else if (layer_nr > 0 || mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT)
    { // only include adhesion only for layers where platform adhesion actually occurs
        // i.e. layers < 0 are for raft, layer 0 is for brim/skirt
        include_adhesion = false;
    }
    if (include_adhesion && mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::NONE)
    {
        ret[mesh_group_settings.get<ExtruderTrain&>("adhesion_extruder_nr").extruder_nr] = true;
        { // process brim/skirt
            for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice->scene.extruders.size(); extruder_nr++)
            {
                if (skirt_brim[extruder_nr].size() > 0)
                {
                    ret[extruder_nr] = true;
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
                if (!support_layer.support_infill_parts.empty())
                {
                    ret[mesh_group_settings.get<ExtruderTrain&>("support_extruder_nr_layer_0").extruder_nr] = true;
                }
            }
            else
            {
                if (!support_layer.support_infill_parts.empty())
                {
                    ret[mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr] = true;
                }
            }
            if (!support_layer.support_bottom.empty())
            {
                ret[mesh_group_settings.get<ExtruderTrain&>("support_bottom_extruder_nr").extruder_nr] = true;
            }
            if (!support_layer.support_roof.empty())
            {
                ret[mesh_group_settings.get<ExtruderTrain&>("support_roof_extruder_nr").extruder_nr] = true;
            }
        }
    }

    if (include_models)
    {
        for (const SliceMeshStorage& mesh : meshes)
        {
            for (unsigned int extruder_nr = 0; extruder_nr < ret.size(); extruder_nr++)
            {
                ret[extruder_nr] = ret[extruder_nr] || mesh.getExtruderIsUsed(extruder_nr, layer_nr);
            }
        }
    }
    return ret;
}

bool SliceDataStorage::getExtruderPrimeBlobEnabled(const size_t extruder_nr) const
{
    if (extruder_nr >= Application::getInstance().current_slice->scene.extruders.size())
    {
        return false;
    }

    const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];
    return train.settings.get<bool>("prime_blob_enable");
}

Polygon SliceDataStorage::getMachineBorder(bool adhesion_offset) const
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;

    Polygon border{};
    switch(mesh_group_settings.get<BuildPlateShape>("machine_shape"))
    {
        case BuildPlateShape::ELLIPTIC:
        {
            //Construct an ellipse to approximate the build volume.
            const coord_t width = machine_size.max.x - machine_size.min.x;
            const coord_t depth = machine_size.max.y - machine_size.min.y;
            constexpr unsigned int circle_resolution = 50;
            for (unsigned int i = 0; i < circle_resolution; i++)
            {
                const double angle = M_PI * 2 * i / circle_resolution;
                border.emplace_back(machine_size.getMiddle().x + std::cos(angle) * width / 2,
                                    machine_size.getMiddle().y + std::sin(angle) * depth / 2);
            }
            break;
        }
        case BuildPlateShape::RECTANGULAR:
        default:
            border = machine_size.flatten().toPolygon();
            break;
    }
    if (!adhesion_offset) {
        return border;
    }

    coord_t adhesion_size = 0; //Make sure there is enough room for the platform adhesion around support.
    const ExtruderTrain& adhesion_extruder = mesh_group_settings.get<ExtruderTrain&>("adhesion_extruder_nr");
    coord_t extra_skirt_line_width = 0;
    const std::vector<bool> is_extruder_used = getExtrudersUsed();
    for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice->scene.extruders.size(); extruder_nr++)
    {
        if (extruder_nr == adhesion_extruder.extruder_nr || !is_extruder_used[extruder_nr]) //Unused extruders and the primary adhesion extruder don't generate an extra skirt line.
        {
            continue;
        }
        const ExtruderTrain& other_extruder = Application::getInstance().current_slice->scene.extruders[extruder_nr];
        extra_skirt_line_width += other_extruder.settings.get<coord_t>("skirt_brim_line_width") * other_extruder.settings.get<Ratio>("initial_layer_line_width_factor");
    }
    switch (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type"))
    {
        case EPlatformAdhesion::BRIM:
            adhesion_size = adhesion_extruder.settings.get<coord_t>("skirt_brim_line_width") * adhesion_extruder.settings.get<Ratio>("initial_layer_line_width_factor") * adhesion_extruder.settings.get<size_t>("brim_line_count") + extra_skirt_line_width;
            break;
        case EPlatformAdhesion::RAFT:
            adhesion_size = adhesion_extruder.settings.get<coord_t>("raft_margin");
            break;
        case EPlatformAdhesion::SKIRT:
            adhesion_size = adhesion_extruder.settings.get<coord_t>("skirt_gap") + adhesion_extruder.settings.get<coord_t>("skirt_brim_line_width") * adhesion_extruder.settings.get<Ratio>("initial_layer_line_width_factor") * adhesion_extruder.settings.get<size_t>("skirt_line_count") + extra_skirt_line_width;
            break;
        case EPlatformAdhesion::NONE:
            adhesion_size = 0;
            break;
        default: //Also use 0.
            log("Unknown platform adhesion type! Please implement the width of the platform adhesion here.");
            break;
    }
    return border.offset(-adhesion_size)[0];
}


void SupportLayer::excludeAreasFromSupportInfillAreas(const Polygons& exclude_polygons, const AABB& exclude_polygons_boundary_box)
{
    // record the indexes that need to be removed and do that after
    std::list<size_t> to_remove_part_indices;  // LIFO for removing

    unsigned int part_count_to_check = support_infill_parts.size(); // note that support_infill_parts.size() changes during the computation below
    for (size_t part_idx = 0; part_idx < part_count_to_check; ++part_idx)
    {
        SupportInfillPart& support_infill_part = support_infill_parts[part_idx];

        // if the areas don't overlap, do nothing
        if (!exclude_polygons_boundary_box.hit(support_infill_part.outline_boundary_box))
        {
            continue;
        }

        Polygons result_polygons = support_infill_part.outline.difference(exclude_polygons);

        // if no smaller parts get generated, this mean this part should be removed.
        if (result_polygons.empty())
        {
            to_remove_part_indices.push_back(part_idx);
            continue;
        }

        std::vector<PolygonsPart> smaller_support_islands = result_polygons.splitIntoParts();

        if (smaller_support_islands.empty())
        { // extra safety guard in case result_polygons consists of too small polygons which are automatically removed in splitIntoParts
            to_remove_part_indices.push_back(part_idx);
            continue;
        }

        // there are one or more smaller parts.
        // we first replace the current part with one of the smaller parts,
        // the rest we add to the support_infill_parts (but after part_count_to_check)
        support_infill_part.outline = smaller_support_islands[0];

        for (size_t support_island_idx = 1; support_island_idx < smaller_support_islands.size(); ++support_island_idx)
        {
            const PolygonsPart& smaller_island = smaller_support_islands[support_island_idx];
            support_infill_parts.emplace_back(smaller_island, support_infill_part.support_line_width, support_infill_part.inset_count_to_generate);
        }
    }

    // remove the ones that need to be removed (LIFO)
    while (!to_remove_part_indices.empty())
    {
        const size_t remove_idx = to_remove_part_indices.back();
        to_remove_part_indices.pop_back();

        if (remove_idx < support_infill_parts.size() - 1)
        { // move last element to the to-be-removed element so that we can erase the last place in the vector
            support_infill_parts[remove_idx] = std::move(support_infill_parts.back());
        }
        support_infill_parts.pop_back(); // always erase last place in the vector
    }
}

} // namespace cura
