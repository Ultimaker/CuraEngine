// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "sliceDataStorage.h"

#include <spdlog/spdlog.h>

#include "Application.h" //To get settings.
#include "ExtruderTrain.h"
#include "FffProcessor.h" //To create a mesh group with if none is provided.
#include "Slice.h"
#include "infill/DensityProvider.h" // for destructor
#include "infill/LightningGenerator.h"
#include "infill/SierpinskiFillProvider.h"
#include "infill/SubDivCube.h" // For the destructor
#include "raft.h"
#include "utils/math.h" //For PI.


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

bool SliceLayerPart::hasWallAtInsetIndex(size_t inset_idx) const
{
    for (const VariableWidthLines& lines : wall_toolpaths)
    {
        for (const ExtrusionLine& line : lines)
        {
            if (line.inset_idx == inset_idx)
            {
                return true;
            }
        }
    }
    return false;
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

SliceMeshStorage::SliceMeshStorage(Mesh* mesh, const size_t slice_layer_count)
    : settings(mesh->settings)
    , mesh_name(mesh->mesh_name)
    , layer_nr_max_filled_layer(0)
    , bounding_box(mesh->getAABB())
    , base_subdiv_cube(nullptr)
    , cross_fill_provider(nullptr)
    , lightning_generator(nullptr)
{
    layers.resize(slice_layer_count);
}


bool SliceMeshStorage::getExtruderIsUsed(const size_t extruder_nr) const
{
    if (settings.get<bool>("anti_overhang_mesh") || settings.get<bool>("support_mesh"))
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
    if (settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr == extruder_nr)
    {
        return true;
    }
    if (settings.get<size_t>("wall_line_count") > 0 && settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr == extruder_nr)
    {
        return true;
    }
    if ((settings.get<size_t>("wall_line_count") > 1 || settings.get<bool>("alternate_extra_perimeter"))
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
    const size_t roofing_layer_count = std::min(settings.get<size_t>("roofing_layer_count"), settings.get<size_t>("top_layers"));
    if (roofing_layer_count > 0 && settings.get<ExtruderTrain&>("roofing_extruder_nr").extruder_nr == extruder_nr)
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
    if (settings.get<bool>("anti_overhang_mesh") || settings.get<bool>("support_mesh"))
    { // object is not printed as object, but as support.
        return false;
    }
    const SliceLayer& layer = layers[layer_nr];
    if (settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr == extruder_nr
        && (settings.get<size_t>("wall_line_count") > 0 || settings.get<size_t>("skin_outline_count") > 0))
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            if ((part.hasWallAtInsetIndex(0)) || ! part.spiral_wall.empty())
            {
                return true;
            }
        }
    }
    if (settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr == extruder_nr
        && layer.openPolyLines.size() > 0)
    {
        return true;
    }
    if ((settings.get<size_t>("wall_line_count") > 1 || settings.get<bool>("alternate_extra_perimeter"))
        && settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr == extruder_nr)
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            if (part.hasWallAtInsetIndex(1))
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
                if (! skin_part.skin_fill.empty())
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
                if (! skin_part.roofing_fill.empty())
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
    return ! settings.get<bool>("infill_mesh") && ! settings.get<bool>("cutting_mesh") && ! settings.get<bool>("anti_overhang_mesh");
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

std::vector<RetractionAndWipeConfig> SliceDataStorage::initializeRetractionAndWipeConfigs()
{
    std::vector<RetractionAndWipeConfig> ret;
    ret.resize(Application::getInstance().current_slice->scene.extruders.size()); // initializes with constructor RetractionConfig()
    return ret;
}

SliceDataStorage::SliceDataStorage()
    : print_layer_count(0)
    , retraction_wipe_config_per_extruder(initializeRetractionAndWipeConfigs())
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

Polygons
    SliceDataStorage::getLayerOutlines(const LayerIndex layer_nr, const bool include_support, const bool include_prime_tower, const bool external_polys_only, const int extruder_nr)
        const
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    if (layer_nr < 0 && layer_nr < -static_cast<LayerIndex>(Raft::getFillerLayerCount()))
    { // when processing raft
        if (include_support && (extruder_nr == -1 || extruder_nr == int(mesh_group_settings.get<ExtruderTrain&>("adhesion_extruder_nr").extruder_nr)))
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
            for (const std::shared_ptr<SliceMeshStorage>& mesh : meshes)
            {
                if (mesh->settings.get<bool>("infill_mesh") || mesh->settings.get<bool>("anti_overhang_mesh")
                    || (extruder_nr != -1 && extruder_nr != int(mesh->settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr)))
                {
                    continue;
                }
                const SliceLayer& layer = mesh->layers[layer_nr];
                layer.getOutlines(total, external_polys_only);
                if (mesh->settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL)
                {
                    total = total.unionPolygons(layer.openPolyLines.offsetPolyLine(MM2INT(0.1)));
                }
            }
        }
        if (include_support && (extruder_nr == -1 || extruder_nr == int(mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr)))
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
        int prime_tower_outer_extruder_nr = primeTower.extruder_order[0];
        if (include_prime_tower && (extruder_nr == -1 || extruder_nr == prime_tower_outer_extruder_nr))
        {
            if (primeTower.enabled)
            {
                total.add(primeTower.getOuterPoly(layer_nr));
            }
        }
        return total;
    }
}

std::vector<bool> SliceDataStorage::getExtrudersUsed() const
{
    std::vector<bool> ret;
    ret.resize(Application::getInstance().current_slice->scene.extruders.size(), false);

    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const EPlatformAdhesion adhesion_type = mesh_group_settings.get<EPlatformAdhesion>("adhesion_type");
    if (adhesion_type == EPlatformAdhesion::SKIRT || adhesion_type == EPlatformAdhesion::BRIM)
    {
        for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice->scene.extruders.size(); extruder_nr++)
        {
            if (! skirt_brim[extruder_nr].empty())
            {
                ret[extruder_nr] = true;
            }
        }
        int skirt_brim_extruder_nr = mesh_group_settings.get<int>("skirt_brim_extruder_nr");
        if (skirt_brim_extruder_nr >= 0)
        {
            ret[skirt_brim_extruder_nr] = true;
        }
    }
    else if (adhesion_type == EPlatformAdhesion::RAFT)
    {
        ret[mesh_group_settings.get<ExtruderTrain&>("raft_base_extruder_nr").extruder_nr] = true;
        const size_t num_interface_layers = mesh_group_settings.get<ExtruderTrain&>("raft_interface_extruder_nr").settings.get<size_t>("raft_interface_layers");
        if (num_interface_layers > 0)
        {
            ret[mesh_group_settings.get<ExtruderTrain&>("raft_interface_extruder_nr").extruder_nr] = true;
        }
        const size_t num_surface_layers = mesh_group_settings.get<ExtruderTrain&>("raft_surface_extruder_nr").settings.get<size_t>("raft_surface_layers");
        if (num_surface_layers > 0)
        {
            ret[mesh_group_settings.get<ExtruderTrain&>("raft_surface_extruder_nr").extruder_nr] = true;
        }
    }

    // TODO: ooze shield, draft shield ..?

    // support
    // support is presupposed to be present...
    for (const std::shared_ptr<SliceMeshStorage>& mesh : meshes)
    {
        if (mesh->settings.get<bool>("support_enable") || mesh->settings.get<bool>("support_mesh"))
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
    for (const std::shared_ptr<SliceMeshStorage>& mesh : meshes)
    {
        for (unsigned int extruder_nr = 0; extruder_nr < ret.size(); extruder_nr++)
        {
            ret[extruder_nr] = ret[extruder_nr] || mesh->getExtruderIsUsed(extruder_nr);
        }
    }
    return ret;
}

std::vector<bool> SliceDataStorage::getExtrudersUsed(const LayerIndex layer_nr) const
{
    const std::vector<ExtruderTrain>& extruders = Application::getInstance().current_slice->scene.extruders;
    std::vector<bool> ret;
    ret.resize(extruders.size(), false);
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const EPlatformAdhesion adhesion_type = mesh_group_settings.get<EPlatformAdhesion>("adhesion_type");

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
    }
    else if (layer_nr > 0 || adhesion_type == EPlatformAdhesion::RAFT)
    { // only include adhesion only for layers where platform adhesion actually occurs
        // i.e. layers < 0 are for raft, layer 0 is for brim/skirt
        include_adhesion = false;
    }

    if (include_adhesion)
    {
        if (layer_nr == 0 && (adhesion_type == EPlatformAdhesion::SKIRT || adhesion_type == EPlatformAdhesion::BRIM))
        {
            for (size_t extruder_nr = 0; extruder_nr < extruders.size(); ++extruder_nr)
            {
                if (! skirt_brim[extruder_nr].empty())
                {
                    ret[extruder_nr] = true;
                }
            }
        }
        if (adhesion_type == EPlatformAdhesion::RAFT)
        {
            const LayerIndex raft_layers = Raft::getTotalExtraLayers();
            if (layer_nr == -raft_layers) // Base layer.
            {
                ret[mesh_group_settings.get<ExtruderTrain&>("raft_base_extruder_nr").extruder_nr] = true;
                // When using a raft, all prime blobs need to be on the lowest layer (the build plate).
                for (size_t extruder_nr = 0; extruder_nr < extruders.size(); ++extruder_nr)
                {
                    if (extruders[extruder_nr].settings.get<bool>("prime_blob_enable"))
                    {
                        ret[extruder_nr] = true;
                    }
                }
            }
            else if (layer_nr == -raft_layers + 1) // Interface layer.
            {
                ret[mesh_group_settings.get<ExtruderTrain&>("raft_interface_extruder_nr").extruder_nr] = true;
            }
            else if (layer_nr < -static_cast<LayerIndex>(Raft::getFillerLayerCount())) // Any of the surface layers.
            {
                ret[mesh_group_settings.get<ExtruderTrain&>("raft_surface_extruder_nr").extruder_nr] = true;
            }
        }
    }

    // TODO: ooze shield, draft shield ..?

    if (include_helper_parts)
    {
        // support
        if (layer_nr < int(support.supportLayers.size()))
        {
            const SupportLayer& support_layer
                = support.supportLayers[std::max(LayerIndex(0), layer_nr)]; // Below layer 0, it's the same as layer 0 (even though it's not stored here).
            if (layer_nr == 0)
            {
                if (! support_layer.support_infill_parts.empty())
                {
                    ret[mesh_group_settings.get<ExtruderTrain&>("support_extruder_nr_layer_0").extruder_nr] = true;
                }
            }
            else
            {
                if (! support_layer.support_infill_parts.empty())
                {
                    ret[mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr] = true;
                }
            }
            if (! support_layer.support_bottom.empty())
            {
                ret[mesh_group_settings.get<ExtruderTrain&>("support_bottom_extruder_nr").extruder_nr] = true;
            }
            if (! support_layer.support_roof.empty())
            {
                ret[mesh_group_settings.get<ExtruderTrain&>("support_roof_extruder_nr").extruder_nr] = true;
            }
        }
    }

    if (include_models)
    {
        for (const std::shared_ptr<SliceMeshStorage>& mesh : meshes)
        {
            for (unsigned int extruder_nr = 0; extruder_nr < ret.size(); extruder_nr++)
            {
                ret[extruder_nr] = ret[extruder_nr] || mesh->getExtruderIsUsed(extruder_nr, layer_nr);
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

Polygons SliceDataStorage::getMachineBorder(int checking_extruder_nr) const
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;

    Polygons border;
    border.emplace_back();
    PolygonRef outline = border.back();
    switch (mesh_group_settings.get<BuildPlateShape>("machine_shape"))
    {
    case BuildPlateShape::ELLIPTIC:
    {
        // Construct an ellipse to approximate the build volume.
        const coord_t width = machine_size.max.x - machine_size.min.x;
        const coord_t depth = machine_size.max.y - machine_size.min.y;
        constexpr unsigned int circle_resolution = 50;
        for (unsigned int i = 0; i < circle_resolution; i++)
        {
            const double angle = M_PI * 2 * i / circle_resolution;
            outline.emplace_back(machine_size.getMiddle().x + std::cos(angle) * width / 2, machine_size.getMiddle().y + std::sin(angle) * depth / 2);
        }
        break;
    }
    case BuildPlateShape::RECTANGULAR:
    default:
        outline = machine_size.flatten().toPolygon();
        break;
    }

    Polygons disallowed_areas = mesh_group_settings.get<Polygons>("machine_disallowed_areas");
    disallowed_areas = disallowed_areas.unionPolygons(); // union overlapping disallowed areas
    for (PolygonRef poly : disallowed_areas)
        for (Point& p : poly)
            p = Point(machine_size.max.x / 2 + p.X, machine_size.max.y / 2 - p.Y); // apparently the frontend stores the disallowed areas in a different coordinate system

    std::vector<bool> extruder_is_used = getExtrudersUsed();

    constexpr coord_t prime_clearance = MM2INT(6.5);
    for (size_t extruder_nr = 0; extruder_nr < extruder_is_used.size(); extruder_nr++)
    {
        if ((checking_extruder_nr != -1 && int(extruder_nr) != checking_extruder_nr) || ! extruder_is_used[extruder_nr])
        {
            continue;
        }
        Settings& extruder_settings = Application::getInstance().current_slice->scene.extruders[extruder_nr].settings;
        if (! (extruder_settings.get<bool>("prime_blob_enable") && mesh_group_settings.get<bool>("extruder_prime_pos_abs")))
        {
            continue;
        }
        Point prime_pos(extruder_settings.get<coord_t>("extruder_prime_pos_x"), extruder_settings.get<coord_t>("extruder_prime_pos_y"));
        if (prime_pos == Point(0, 0))
        {
            continue; // Ignore extruder prime position if it is not set.
        }
        Point translation(extruder_settings.get<coord_t>("machine_nozzle_offset_x"), extruder_settings.get<coord_t>("machine_nozzle_offset_y"));
        prime_pos -= translation;
        Polygons prime_polygons;
        prime_polygons.emplace_back(PolygonUtils::makeCircle(prime_pos, prime_clearance, M_PI / 32));
        disallowed_areas = disallowed_areas.unionPolygons(prime_polygons);
    }

    Polygons disallowed_all_extruders;
    bool first = true;
    for (size_t extruder_nr = 0; extruder_nr < extruder_is_used.size(); extruder_nr++)
    {
        if ((checking_extruder_nr != -1 && int(extruder_nr) != checking_extruder_nr) || ! extruder_is_used[extruder_nr])
        {
            continue;
        }
        Settings& extruder_settings = Application::getInstance().current_slice->scene.extruders[extruder_nr].settings;
        Point translation(extruder_settings.get<coord_t>("machine_nozzle_offset_x"), extruder_settings.get<coord_t>("machine_nozzle_offset_y"));
        Polygons extruder_border = disallowed_areas;
        extruder_border.translate(translation);
        if (first)
        {
            disallowed_all_extruders = extruder_border;
            first = false;
        }
        else
        {
            disallowed_all_extruders = disallowed_all_extruders.unionPolygons(extruder_border);
        }
    }
    disallowed_all_extruders.processEvenOdd(ClipperLib::pftNonZero); // prevent overlapping disallowed areas from XORing

    Polygons border_all_extruders = border; // each extruders border areas must be limited to the global border, which is the union of all extruders borders
    if (mesh_group_settings.has("nozzle_offsetting_for_disallowed_areas") && mesh_group_settings.get<bool>("nozzle_offsetting_for_disallowed_areas"))
    {
        for (size_t extruder_nr = 0; extruder_nr < extruder_is_used.size(); extruder_nr++)
        {
            if ((checking_extruder_nr != -1 && int(extruder_nr) != checking_extruder_nr) || ! extruder_is_used[extruder_nr])
            {
                continue;
            }
            Settings& extruder_settings = Application::getInstance().current_slice->scene.extruders[extruder_nr].settings;
            Point translation(extruder_settings.get<coord_t>("machine_nozzle_offset_x"), extruder_settings.get<coord_t>("machine_nozzle_offset_y"));
            for (size_t other_extruder_nr = 0; other_extruder_nr < extruder_is_used.size(); other_extruder_nr++)
            {
                // NOTE: the other extruder doesn't have to be used. Since the global border is the union of all extruders borders also unused extruders must be taken into account.
                if (other_extruder_nr == extruder_nr)
                {
                    continue;
                }
                Settings& other_extruder_settings = Application::getInstance().current_slice->scene.extruders[other_extruder_nr].settings;
                Point other_translation(other_extruder_settings.get<coord_t>("machine_nozzle_offset_x"), other_extruder_settings.get<coord_t>("machine_nozzle_offset_y"));
                Polygons translated_border = border;
                translated_border.translate(translation - other_translation);
                border_all_extruders = border_all_extruders.intersection(translated_border);
            }
        }
    }

    border = border_all_extruders.difference(disallowed_all_extruders);
    return border;
}


void SupportLayer::excludeAreasFromSupportInfillAreas(const Polygons& exclude_polygons, const AABB& exclude_polygons_boundary_box)
{
    // record the indexes that need to be removed and do that after
    std::list<size_t> to_remove_part_indices; // LIFO for removing

    unsigned int part_count_to_check = support_infill_parts.size(); // note that support_infill_parts.size() changes during the computation below
    for (size_t part_idx = 0; part_idx < part_count_to_check; ++part_idx)
    {
        SupportInfillPart& support_infill_part = support_infill_parts[part_idx];

        // if the areas don't overlap, do nothing
        if (! exclude_polygons_boundary_box.hit(support_infill_part.outline_boundary_box))
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
    while (! to_remove_part_indices.empty())
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

void SupportLayer::fillInfillParts(
    const LayerIndex layer_nr,
    const std::vector<Polygons>& support_fill_per_layer,
    const coord_t support_line_width,
    const coord_t wall_line_count,
    const coord_t grow_layer_above /*has default 0*/,
    const bool unionAll /*has default false*/)
{
    const Polygons& support_this_layer = support_fill_per_layer[layer_nr];
    const Polygons& support_layer_above
        = (layer_nr + 1) >= support_fill_per_layer.size() || layer_nr <= 0 ? Polygons() : support_fill_per_layer[layer_nr + 1].offset(grow_layer_above);
    const auto all_support_areas_in_layer = { support_this_layer.difference(support_layer_above), support_this_layer.intersection(support_layer_above) };
    bool use_fractional_config = true;
    for (auto& support_areas : all_support_areas_in_layer)
    {
        for (const PolygonsPart& island_outline : support_areas.splitIntoParts(unionAll))
        {
            support_infill_parts.emplace_back(island_outline, support_line_width, use_fractional_config, wall_line_count);
        }
        use_fractional_config = false;
    }
}

} // namespace cura
