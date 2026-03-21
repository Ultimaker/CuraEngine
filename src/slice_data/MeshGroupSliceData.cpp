// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "slice_data/MeshGroupSliceData.h"

#include <numbers>

#include "Application.h" //To get settings.
#include "ExtruderTrain.h"
#include "PrimeTower/PrimeTower.h"
#include "Slice.h"
#include "raft.h"
#include "slice_data/MeshSliceData.h"


namespace cura
{

std::vector<RetractionAndWipeConfig> MeshGroupSliceData::initializeRetractionAndWipeConfigs()
{
    std::vector<RetractionAndWipeConfig> ret;
    ret.resize(Application::getInstance().current_slice_->scene.extruders.size()); // initializes with constructor RetractionConfig()
    return ret;
}

MeshGroupSliceData::MeshGroupSliceData(const Settings& settings)
    : settings_(settings)
    , print_layer_count(0)
    , retraction_wipe_config_per_extruder(initializeRetractionAndWipeConfigs())
    , max_print_height_second_to_last_extruder(-1)
{
    Point3LL machine_max(settings_.get<coord_t>("machine_width"), settings_.get<coord_t>("machine_depth"), settings_.get<coord_t>("machine_height"));
    Point3LL machine_min(0, 0, 0);
    if (settings_.get<bool>("machine_center_is_zero"))
    {
        machine_max /= 2;
        machine_min -= machine_max;
    }
    machine_size.include(machine_min);
    machine_size.include(machine_max);
}

MeshGroupSliceData::~MeshGroupSliceData()
{
    delete prime_tower_;
}

Shape MeshGroupSliceData::getLayerOutlines(
    const LayerIndex layer_nr,
    const bool include_support,
    const bool include_prime_tower,
    const bool external_polys_only,
    const int extruder_nr,
    const bool include_models) const
{
    const auto layer_type = Raft::getLayerType(layer_nr);
    switch (layer_type)
    {
    case Raft::LayerType::RaftBase:
    case Raft::LayerType::RaftInterface:
    case Raft::LayerType::RaftSurface:
    {
        const Shape* raftOutline;
        bool use_current_extruder_for_raft = extruder_nr == -1;

        switch (layer_type)
        {
        case Raft::LayerType::RaftBase:
            raftOutline = &raft_base_outline;
            use_current_extruder_for_raft |= extruder_nr == int(settings_.get<ExtruderTrain&>("raft_base_extruder_nr").extruder_nr_);
            break;
        case Raft::LayerType::RaftInterface:
            raftOutline = &raft_interface_outline;
            use_current_extruder_for_raft |= extruder_nr == int(settings_.get<ExtruderTrain&>("raft_interface_extruder_nr").extruder_nr_);
            break;
        case Raft::LayerType::RaftSurface:
            raftOutline = &raft_surface_outline;
            use_current_extruder_for_raft |= extruder_nr == int(settings_.get<ExtruderTrain&>("raft_surface_extruder_nr").extruder_nr_);
            break;
        default:
            assert(false && "unreachable due to outer switch statement");
            return Shape();
        }

        if (include_support && use_current_extruder_for_raft)
        {
            if (external_polys_only)
            {
                std::vector<SingleShape> parts = raftOutline->splitIntoParts();
                Shape result;
                for (SingleShape& part : parts)
                {
                    result.push_back(part.outerPolygon());
                }
                return result;
            }
            else
            {
                return *raftOutline;
            }
        }
        else
        {
            return Shape();
        }
        break;
    }
    case Raft::LayerType::Airgap:
    case Raft::LayerType::Model:
    {
        Shape total;
        if (include_models && layer_nr >= 0)
        {
            for (const std::shared_ptr<MeshSliceData>& mesh : meshes)
            {
                if (mesh->settings.get<bool>("infill_mesh") || mesh->settings.get<bool>("anti_overhang_mesh")
                    || (extruder_nr != -1 && extruder_nr != int(mesh->settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr_)))
                {
                    continue;
                }
                const SliceLayer& layer = mesh->layers[layer_nr];
                layer.getOutlines(total, external_polys_only);
                if (mesh->settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL)
                {
                    total = total.unionPolygons(layer.open_polylines.offset(MM2INT(0.1)));
                }
            }
        }
        if (include_support && (extruder_nr == -1 || extruder_nr == int(settings_.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr_)))
        {
            const SupportLayer& support_layer = support.supportLayers[std::max(LayerIndex(0), layer_nr)];
            if (support.generated)
            {
                for (const SupportInfillPart& support_infill_part : support_layer.support_infill_parts)
                {
                    total.push_back(support_infill_part.outline_);
                }
                total.push_back(support_layer.support_bottom);
                total.push_back(support_layer.support_roof);
            }
        }
        if (include_prime_tower && prime_tower_)
        {
            total.push_back(prime_tower_->getOccupiedOutline(layer_nr));
        }
        return total;
    }
    default:
        assert(false && "unreachable as switch statement is exhaustive");
        return Shape();
    }
}

AABB3D MeshGroupSliceData::getModelBoundingBox() const
{
    AABB3D bounding_box;
    for (const auto& mesh : meshes)
    {
        bounding_box.include(mesh->bounding_box);
    }
    return bounding_box;
}

std::vector<bool> MeshGroupSliceData::getExtrudersUsed() const
{
    std::vector<bool> ret;
    ret.resize(Application::getInstance().current_slice_->scene.extruders.size(), false);

    // set all the false to start, we set them to true if used
    for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice_->scene.extruders.size(); extruder_nr++)
    {
        ret[extruder_nr] = false;
    }

    const EPlatformAdhesion adhesion_type = settings_.get<EPlatformAdhesion>("adhesion_type");
    if (adhesion_type == EPlatformAdhesion::SKIRT || adhesion_type == EPlatformAdhesion::BRIM)
    {
        for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice_->scene.extruders.size(); extruder_nr++)
        {
            if (! skirt_brim[extruder_nr].empty())
            {
                ret[extruder_nr] = true;
            }
        }
        int skirt_brim_extruder_nr = settings_.get<int>("skirt_brim_extruder_nr");
        if (skirt_brim_extruder_nr >= 0)
        {
            ret[skirt_brim_extruder_nr] = true;
        }
    }
    else if (adhesion_type == EPlatformAdhesion::RAFT)
    {
        ret[settings_.get<ExtruderTrain&>("raft_base_extruder_nr").extruder_nr_] = true;
        const size_t num_interface_layers = settings_.get<ExtruderTrain&>("raft_interface_extruder_nr").settings_.get<size_t>("raft_interface_layers");
        if (num_interface_layers > 0)
        {
            ret[settings_.get<ExtruderTrain&>("raft_interface_extruder_nr").extruder_nr_] = true;
        }
        const size_t num_surface_layers = settings_.get<ExtruderTrain&>("raft_surface_extruder_nr").settings_.get<size_t>("raft_surface_layers");
        if (num_surface_layers > 0)
        {
            ret[settings_.get<ExtruderTrain&>("raft_surface_extruder_nr").extruder_nr_] = true;
        }
    }

    // TODO: ooze shield, draft shield ..?

    // support
    // support is presupposed to be present...
    for (const std::shared_ptr<MeshSliceData>& mesh : meshes)
    {
        if (mesh->settings.get<bool>("support_enable") || mesh->settings.get<bool>("support_mesh"))
        {
            ret[settings_.get<ExtruderTrain&>("support_extruder_nr_layer_0").extruder_nr_] = true;
            ret[settings_.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr_] = true;
            if (settings_.get<bool>("support_roof_enable"))
            {
                ret[settings_.get<ExtruderTrain&>("support_roof_extruder_nr").extruder_nr_] = true;
            }
            if (settings_.get<bool>("support_bottom_enable"))
            {
                ret[settings_.get<ExtruderTrain&>("support_bottom_extruder_nr").extruder_nr_] = true;
            }
        }
    }

    // all meshes are presupposed to actually have content
    for (const std::shared_ptr<MeshSliceData>& mesh : meshes)
    {
        for (unsigned int extruder_nr = 0; extruder_nr < ret.size(); extruder_nr++)
        {
            ret[extruder_nr] = ret[extruder_nr] || mesh->getExtruderIsUsed(extruder_nr);
        }
    }
    return ret;
}

std::vector<bool> MeshGroupSliceData::getExtrudersUsed(const LayerIndex layer_nr) const
{
    const std::vector<ExtruderTrain>& extruders = Application::getInstance().current_slice_->scene.extruders;
    std::vector<bool> ret;
    ret.resize(extruders.size(), false);
    const EPlatformAdhesion adhesion_type = settings_.get<EPlatformAdhesion>("adhesion_type");

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
            const Raft::LayerType layer_type = Raft::getLayerType(layer_nr);
            if (layer_type == Raft::RaftBase)
            {
                ret[settings_.get<ExtruderTrain&>("raft_base_extruder_nr").extruder_nr_] = true;
                // When using a raft, all prime blobs need to be on the lowest layer (the build plate).
                for (size_t extruder_nr = 0; extruder_nr < extruders.size(); ++extruder_nr)
                {
                    if (extruders[extruder_nr].settings_.get<bool>("prime_blob_enable"))
                    {
                        ret[extruder_nr] = true;
                    }
                }
            }
            else if (layer_type == Raft::RaftInterface)
            {
                ret[settings_.get<ExtruderTrain&>("raft_interface_extruder_nr").extruder_nr_] = true;
            }
            else if (layer_type == Raft::RaftSurface)
            {
                ret[settings_.get<ExtruderTrain&>("raft_surface_extruder_nr").extruder_nr_] = true;
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
                    ret[settings_.get<ExtruderTrain&>("support_extruder_nr_layer_0").extruder_nr_] = true;
                }
            }
            else
            {
                if (! support_layer.support_infill_parts.empty())
                {
                    ret[settings_.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr_] = true;
                }
            }
            if (! support_layer.support_bottom.empty())
            {
                ret[settings_.get<ExtruderTrain&>("support_bottom_extruder_nr").extruder_nr_] = true;
            }
            if (! support_layer.support_roof.empty())
            {
                ret[settings_.get<ExtruderTrain&>("support_roof_extruder_nr").extruder_nr_] = true;
            }
        }
    }

    if (include_models)
    {
        for (const std::shared_ptr<MeshSliceData>& mesh : meshes)
        {
            for (unsigned int extruder_nr = 0; extruder_nr < ret.size(); extruder_nr++)
            {
                ret[extruder_nr] = ret[extruder_nr] || mesh->getExtruderIsUsed(extruder_nr, layer_nr);
            }
        }
    }
    return ret;
}

bool MeshGroupSliceData::getExtruderPrimeBlobEnabled(const size_t extruder_nr) const
{
    if (extruder_nr >= Application::getInstance().current_slice_->scene.extruders.size())
    {
        return false;
    }

    const ExtruderTrain& train = Application::getInstance().current_slice_->scene.extruders[extruder_nr];
    return train.settings_.get<bool>("prime_blob_enable");
}

Shape MeshGroupSliceData::getMachineBorder(int checking_extruder_nr) const
{
    Shape border = getRawMachineBorder();

    Shape disallowed_areas = settings_.get<Shape>("machine_disallowed_areas");
    disallowed_areas = disallowed_areas.unionPolygons(); // union overlapping disallowed areas

    // The disallowed areas are expressed in buildplate-centered coordinates, but the models
    // may be expressed in front-left-centered coordinantes, so in this case we need to translate them
    if (! settings_.get<bool>("machine_center_is_zero"))
    {
        for (Polygon& poly : disallowed_areas)
        {
            for (Point2LL& p : poly)
            {
                p = Point2LL(machine_size.max_.x_ / 2 + p.X, machine_size.max_.y_ / 2 - p.Y);
            }
        }
    }

    std::vector<bool> extruder_is_used = getExtrudersUsed();

    constexpr coord_t prime_clearance = MM2INT(6.5);
    for (size_t extruder_nr = 0; extruder_nr < extruder_is_used.size(); extruder_nr++)
    {
        if ((checking_extruder_nr != -1 && int(extruder_nr) != checking_extruder_nr) || ! extruder_is_used[extruder_nr])
        {
            continue;
        }
        const Settings& extruder_settings = Application::getInstance().current_slice_->scene.extruders[extruder_nr].settings_;
        if (! (extruder_settings.get<bool>("prime_blob_enable") && settings_.get<bool>("extruder_prime_pos_abs")))
        {
            continue;
        }
        Point2LL prime_pos(extruder_settings.get<coord_t>("extruder_prime_pos_x"), extruder_settings.get<coord_t>("extruder_prime_pos_y"));
        if (prime_pos == Point2LL(0, 0))
        {
            continue; // Ignore extruder prime position if it is not set.
        }
        Point2LL translation(extruder_settings.get<coord_t>("machine_nozzle_offset_x"), extruder_settings.get<coord_t>("machine_nozzle_offset_y"));
        prime_pos -= translation;
        Polygon prime_polygon = PolygonUtils::makeDisc(prime_pos, prime_clearance, 64);
        disallowed_areas = disallowed_areas.unionPolygons(prime_polygon);
    }

    Shape disallowed_all_extruders;
    bool first = true;
    for (size_t extruder_nr = 0; extruder_nr < extruder_is_used.size(); extruder_nr++)
    {
        if ((checking_extruder_nr != -1 && int(extruder_nr) != checking_extruder_nr) || ! extruder_is_used[extruder_nr])
        {
            continue;
        }
        Settings& extruder_settings = Application::getInstance().current_slice_->scene.extruders[extruder_nr].settings_;
        Point2LL translation(extruder_settings.get<coord_t>("machine_nozzle_offset_x"), extruder_settings.get<coord_t>("machine_nozzle_offset_y"));
        Shape extruder_border = disallowed_areas;
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
    disallowed_all_extruders = disallowed_all_extruders.processEvenOdd(ClipperLib::pftNonZero); // prevent overlapping disallowed areas from XORing

    Shape border_all_extruders = border; // each extruders border areas must be limited to the global border, which is the union of all extruders borders
    if (settings_.has("nozzle_offsetting_for_disallowed_areas") && settings_.get<bool>("nozzle_offsetting_for_disallowed_areas"))
    {
        for (size_t extruder_nr = 0; extruder_nr < extruder_is_used.size(); extruder_nr++)
        {
            if ((checking_extruder_nr != -1 && int(extruder_nr) != checking_extruder_nr) || ! extruder_is_used[extruder_nr])
            {
                continue;
            }
            Settings& extruder_settings = Application::getInstance().current_slice_->scene.extruders[extruder_nr].settings_;
            Point2LL translation(extruder_settings.get<coord_t>("machine_nozzle_offset_x"), extruder_settings.get<coord_t>("machine_nozzle_offset_y"));
            for (size_t other_extruder_nr = 0; other_extruder_nr < extruder_is_used.size(); other_extruder_nr++)
            {
                // NOTE: the other extruder doesn't have to be used. Since the global border is the union of all extruders borders also unused extruders must be taken into account.
                if (other_extruder_nr == extruder_nr)
                {
                    continue;
                }
                Settings& other_extruder_settings = Application::getInstance().current_slice_->scene.extruders[other_extruder_nr].settings_;
                Point2LL other_translation(other_extruder_settings.get<coord_t>("machine_nozzle_offset_x"), other_extruder_settings.get<coord_t>("machine_nozzle_offset_y"));
                Shape translated_border = border;
                translated_border.translate(translation - other_translation);
                border_all_extruders = border_all_extruders.intersection(translated_border);
            }
        }
    }

    border = border_all_extruders.difference(disallowed_all_extruders);
    return border;
}

Shape MeshGroupSliceData::getRawMachineBorder() const
{
    Shape border;
    border.emplace_back();
    Polygon& outline = border.back();
    switch (settings_.get<BuildPlateShape>("machine_shape"))
    {
    case BuildPlateShape::ELLIPTIC:
    {
        // Construct an ellipse to approximate the build volume.
        const coord_t width = machine_size.max_.x_ - machine_size.min_.x_;
        const coord_t depth = machine_size.max_.y_ - machine_size.min_.y_;
        constexpr unsigned int circle_resolution = 50;
        for (unsigned int i = 0; i < circle_resolution; i++)
        {
            const double angle = std::numbers::pi * 2 * i / circle_resolution;
            outline.emplace_back(machine_size.getMiddle().x_ + std::cos(angle) * width / 2, machine_size.getMiddle().y_ + std::sin(angle) * depth / 2);
        }
        break;
    }
    case BuildPlateShape::RECTANGULAR:
    default:
        outline = machine_size.flatten().toPolygon();
        break;
    }

    return border;
}

void MeshGroupSliceData::initializePrimeTower()
{
    prime_tower_ = PrimeTower::createPrimeTower(*this);
}

} // namespace cura
