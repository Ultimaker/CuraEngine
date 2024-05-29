// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PrimeTower/PrimeTower.h"

#include <algorithm>
#include <limits>
#include <numbers>

#include <spdlog/spdlog.h>

#include "Application.h" //To get settings.
#include "ExtruderTrain.h"
#include "LayerPlan.h"
#include "PrimeTower/PrimeTowerInterleaved.h"
#include "PrimeTower/PrimeTowerNormal.h"
#include "Scene.h"
#include "Slice.h"
#include "gcodeExport.h"
#include "infill.h"
#include "raft.h"
#include "sliceDataStorage.h"

#define CIRCLE_RESOLUTION 32 // The number of vertices in each circle.
#define ARC_RESOLUTION 4 // The number of segments in each arc of a wheel


namespace cura
{

PrimeTower::PrimeTower(SliceDataStorage& storage, size_t extruder_count)
    : wipe_from_middle_(false)
    , extruder_order_(extruder_count)
{
    const Scene& scene = Application::getInstance().current_slice_->scene;

    // First make a basic list of used extruders numbers
    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        extruder_order_[extruder_nr] = extruder_nr;
    }

    // Then sort from high adhesion to low adhesion.
    std::stable_sort(
        extruder_order_.begin(),
        extruder_order_.end(),
        [&scene](const unsigned int& extruder_nr_a, const unsigned int& extruder_nr_b) -> bool
        {
            const Ratio adhesion_a = scene.extruders[extruder_nr_a].settings_.get<Ratio>("material_adhesion_tendency");
            const Ratio adhesion_b = scene.extruders[extruder_nr_b].settings_.get<Ratio>("material_adhesion_tendency");
            return adhesion_a < adhesion_b;
        });
}

void PrimeTower::generateGroundpoly()
{
    const Scene& scene = Application::getInstance().current_slice_->scene;
    const Settings& mesh_group_settings = scene.current_mesh_group->settings;
    const coord_t tower_size = mesh_group_settings.get<coord_t>("prime_tower_size");

    const coord_t x = mesh_group_settings.get<coord_t>("prime_tower_position_x");
    const coord_t y = mesh_group_settings.get<coord_t>("prime_tower_position_y");
    const coord_t tower_radius = tower_size / 2;
    outer_poly_.push_back(PolygonUtils::makeCircle(Point2LL(x - tower_radius, y + tower_radius), tower_radius, TAU / CIRCLE_RESOLUTION));
    middle_ = Point2LL(x - tower_size / 2, y + tower_size / 2);

    post_wipe_point_ = Point2LL(x - tower_size / 2, y + tower_size / 2);
}

void PrimeTower::generatePaths()
{
    generateGroundpoly();

    std::vector<coord_t> cumulative_insets;
    generateDenseInfill(cumulative_insets);

    generateStartLocations();

    generateSparseInfill(cumulative_insets);
}

void PrimeTower::generateDenseInfill(std::vector<coord_t>& cumulative_insets)
{
    const Scene& scene = Application::getInstance().current_slice_->scene;
    const Settings& mesh_group_settings = scene.current_mesh_group->settings;
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
    const bool base_enabled = mesh_group_settings.get<bool>("prime_tower_brim_enable");
    const coord_t base_extra_radius = scene.settings.get<coord_t>("prime_tower_base_size");
    const bool has_raft = mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT;
    const coord_t base_height = std::max(scene.settings.get<coord_t>("prime_tower_base_height"), has_raft ? layer_height : 0);
    const double base_curve_magnitude = mesh_group_settings.get<double>("prime_tower_base_curve_magnitude");

    for (size_t extruder_nr : extruder_order_)
    {
        // By default, add empty moves for every extruder
        prime_moves_[extruder_nr];
        base_extra_moves_[extruder_nr].init(true);
        inset_extra_moves_[extruder_nr];
    }
    outer_poly_base_.init(true);

    coord_t cumulative_inset = 0; // Each tower shape is going to be printed inside the other. This is the inset we're doing for each extruder.
    for (size_t extruder_nr : extruder_order_)
    {
        const coord_t line_width = scene.extruders[extruder_nr].settings_.get<coord_t>("prime_tower_line_width");
        const coord_t required_volume = MM3_2INT(scene.extruders[extruder_nr].settings_.get<double>("prime_tower_min_volume"));
        const Ratio flow = scene.extruders[extruder_nr].settings_.get<Ratio>("prime_tower_flow");
        coord_t current_volume = 0;
        Shape& prime_moves = prime_moves_[extruder_nr];

        // Create the walls of the prime tower.
        unsigned int wall_nr = 0;
        for (; current_volume < required_volume; wall_nr++)
        {
            // Create a new polygon with an offset from the outer polygon.
            Shape polygons = outer_poly_.offset(-cumulative_inset - wall_nr * line_width - line_width / 2);
            prime_moves.push_back(polygons);
            current_volume += polygons.length() * line_width * layer_height * flow;
            if (polygons.empty()) // Don't continue. We won't ever reach the required volume because it doesn't fit.
            {
                break;
            }
        }

        // Generate the base outside extra rings
        if (requiresBaseExtraPrint(extruder_nr) && (base_enabled || has_raft) && base_extra_radius > 0 && base_height > 0)
        {
            for (coord_t z = 0; z < base_height; z += layer_height)
            {
                double brim_radius_factor = std::pow((1.0 - static_cast<double>(z) / base_height), base_curve_magnitude);
                coord_t extra_radius = base_extra_radius * brim_radius_factor;
                size_t extra_rings = extra_radius / line_width;
                if (extra_rings == 0)
                {
                    break;
                }
                extra_radius = line_width * extra_rings;
                outer_poly_base_.push_back(outer_poly_.offset(extra_radius));
                base_extra_moves_[extruder_nr].push_back(PolygonUtils::generateOutset(outer_poly_, extra_rings, line_width));
            }
        }

        cumulative_inset += wall_nr * line_width;
        cumulative_insets.push_back(cumulative_inset);
    }

    // Now we have the total cumulative inset, generate the base inside extra rings
    for (size_t extruder_nr : extruder_order_)
    {
        if (requiresFirstLayerExtraInnerPrint(extruder_nr))
        {
            const coord_t line_width = scene.extruders[extruder_nr].settings_.get<coord_t>("prime_tower_line_width");
            Shape pattern = PolygonUtils::generateInset(outer_poly_, line_width, cumulative_inset);
            if (! pattern.empty())
            {
                inset_extra_moves_[extruder_nr].push_back(pattern);
            }
        }
    }
}

void PrimeTower::generateSparseInfill(const std::vector<coord_t>& cumulative_insets)
{
    const Scene& scene = Application::getInstance().current_slice_->scene;
    const Settings& mesh_group_settings = scene.current_mesh_group->settings;

    // Pre-compute radiuses of each extruder ring
    std::vector<coord_t> rings_radii;
    const coord_t tower_size = mesh_group_settings.get<coord_t>("prime_tower_size");
    const coord_t tower_radius = tower_size / 2;

    rings_radii.push_back(tower_radius);
    for (const coord_t& cumulative_inset : cumulative_insets)
    {
        rings_radii.push_back(tower_radius - cumulative_inset);
    }

    sparse_pattern_per_extruders_ = generateSparseInfillImpl(rings_radii);
}

Shape PrimeTower::generatePath_sparseInfill(
    const size_t first_extruder_idx,
    const size_t last_extruder_idx,
    const std::vector<coord_t>& rings_radii,
    const coord_t line_width,
    const size_t actual_extruder_nr) const
{
    const Scene& scene = Application::getInstance().current_slice_->scene;
    const coord_t max_bridging_distance = scene.extruders[actual_extruder_nr].settings_.get<coord_t>("prime_tower_max_bridging_distance");
    const coord_t outer_radius = rings_radii[first_extruder_idx];
    const coord_t inner_radius = rings_radii[last_extruder_idx + 1];
    const coord_t radius_delta = outer_radius - inner_radius;
    const coord_t semi_line_width = line_width / 2;

    Shape pattern;

    // Split ring according to max bridging distance
    const size_t nb_rings = std::ceil(static_cast<float>(radius_delta) / max_bridging_distance);
    if (nb_rings)
    {
        const coord_t actual_radius_step = radius_delta / nb_rings;

        for (size_t i = 0; i < nb_rings; ++i)
        {
            const coord_t ring_inner_radius = (inner_radius + i * actual_radius_step) + semi_line_width;
            const coord_t ring_outer_radius = (inner_radius + (i + 1) * actual_radius_step) - semi_line_width;

            const size_t semi_nb_spokes = std::ceil((std::numbers::pi * ring_outer_radius) / max_bridging_distance);

            pattern.push_back(PolygonUtils::makeWheel(middle_, ring_inner_radius, ring_outer_radius, semi_nb_spokes, ARC_RESOLUTION));
        }
    }

    return pattern;
}

void PrimeTower::generateStartLocations()
{
    // Evenly spread out a number of dots along the prime tower's outline. This is done for the complete outline,
    // so use the same start and end segments for this.
    PolygonsPointIndex segment_start = PolygonsPointIndex(&outer_poly_, 0, 0);
    PolygonsPointIndex segment_end = segment_start;

    PolygonUtils::spreadDots(segment_start, segment_end, number_of_prime_tower_start_locations_, prime_tower_start_locations_);
}

void PrimeTower::addToGcode(
    const SliceDataStorage& storage,
    LayerPlan& gcode_layer,
    const std::vector<ExtruderUse>& required_extruder_prime,
    const size_t prev_extruder_nr,
    const size_t new_extruder_nr) const
{
    if (gcode_layer.getPrimeTowerIsPlanned(new_extruder_nr))
    { // don't print the prime tower if it has been printed already with this extruder.
        return;
    }

    const LayerIndex layer_nr = gcode_layer.getLayerNr();
    if (layer_nr > storage.max_print_height_second_to_last_extruder + 1)
    {
        return;
    }

    bool post_wipe = Application::getInstance().current_slice_->scene.extruders[prev_extruder_nr].settings_.get<bool>("prime_tower_wipe_enabled");

    // Do not wipe on the first layer, we will generate non-hollow prime tower there for better bed adhesion.
    if (prev_extruder_nr == new_extruder_nr || layer_nr == 0)
    {
        post_wipe = false;
    }

    auto extruder_iterator = std::find_if(
        required_extruder_prime.begin(),
        required_extruder_prime.end(),
        [new_extruder_nr](const ExtruderUse& extruder_use)
        {
            return extruder_use.extruder_nr == new_extruder_nr;
        });

    if (extruder_iterator == required_extruder_prime.end())
    {
        // Extruder is not used on this layer
        return;
    }

    size_t new_extruder_idx;
    auto iterator = std::find(extruder_order_.begin(), extruder_order_.end(), new_extruder_nr);
    if (iterator != extruder_order_.end())
    {
        new_extruder_idx = iterator - extruder_order_.begin();
    }
    else
    {
        // Given extruder nr is not registered ?!
        return;
    }

    std::vector<size_t> extra_primed_extruders_idx;

    switch (extruder_iterator->prime)
    {
    case ExtruderPrime::None:
        processExtruderNoPrime(new_extruder_nr, gcode_layer);
        break;

    case ExtruderPrime::Sparse:
        gotoStartLocation(gcode_layer, new_extruder_nr);
        extra_primed_extruders_idx = findExtrudersSparseInfill(gcode_layer, required_extruder_prime, { new_extruder_idx });
        addToGcode_sparseInfill(gcode_layer, extra_primed_extruders_idx, new_extruder_nr);
        break;

    case ExtruderPrime::Prime:
        gotoStartLocation(gcode_layer, new_extruder_nr);

        addToGcode_denseInfill(gcode_layer, new_extruder_nr);
        gcode_layer.setPrimeTowerIsPlanned(new_extruder_nr);

        if (gcode_layer.getLayerNr() <= storage.max_print_height_second_to_last_extruder)
        {
            // Whatever happens before and after, use the current extruder to prime all the non-required extruders now
            extra_primed_extruders_idx = findExtrudersSparseInfill(gcode_layer, required_extruder_prime);
            if (! extra_primed_extruders_idx.empty())
            {
                addToGcode_sparseInfill(gcode_layer, extra_primed_extruders_idx, new_extruder_nr);
            }
        }
        break;
    }

    if (! gcode_layer.getPrimeTowerBaseIsPlanned() && addToGcode_base(gcode_layer, new_extruder_nr))
    {
        gcode_layer.setPrimeTowerBaseIsPlanned();
    }

    if (! gcode_layer.getPrimeTowerInsetIsPlanned() && addToGcode_inset(gcode_layer, new_extruder_nr))
    {
        gcode_layer.setPrimeTowerInsetIsPlanned();
    }

    for (const size_t& primed_extruder_idx : extra_primed_extruders_idx)
    {
        gcode_layer.setPrimeTowerIsPlanned(extruder_order_.at(primed_extruder_idx));
    }

    // post-wipe:
    if (post_wipe)
    {
        // Make sure we wipe the old extruder on the prime tower.
        const Settings& previous_settings = Application::getInstance().current_slice_->scene.extruders[prev_extruder_nr].settings_;
        const Point2LL previous_nozzle_offset = Point2LL(previous_settings.get<coord_t>("machine_nozzle_offset_x"), previous_settings.get<coord_t>("machine_nozzle_offset_y"));
        const Settings& new_settings = Application::getInstance().current_slice_->scene.extruders[new_extruder_nr].settings_;
        const Point2LL new_nozzle_offset = Point2LL(new_settings.get<coord_t>("machine_nozzle_offset_x"), new_settings.get<coord_t>("machine_nozzle_offset_y"));
        gcode_layer.addTravel(post_wipe_point_ - previous_nozzle_offset + new_nozzle_offset);
    }
}

void PrimeTower::addToGcode_denseInfill(LayerPlan& gcode_layer, const size_t extruder_nr) const
{
    const size_t raft_total_extra_layers = Raft::getTotalExtraLayers();
    const bool adhesion_raft = raft_total_extra_layers > 0;
    LayerIndex absolute_layer_number = gcode_layer.getLayerNr() + raft_total_extra_layers;

    if (! adhesion_raft || absolute_layer_number > 0)
    {
        // Actual prime pattern
        const GCodePathConfig& config = gcode_layer.configs_storage_.prime_tower_config_per_extruder[extruder_nr];
        const Shape& pattern = prime_moves_.at(extruder_nr);
        gcode_layer.addPolygonsByOptimizer(pattern, config);
    }
}

bool PrimeTower::addToGcode_base(LayerPlan& gcode_layer, const size_t extruder_nr) const
{
    const LayerVector<Shape>& pattern_extra_brim = base_extra_moves_.at(extruder_nr);
    auto iterator = pattern_extra_brim.iterator_at(gcode_layer.getLayerNr());
    if (iterator != pattern_extra_brim.end())
    {
        // Extra rings for stronger base
        const Shape& pattern = *iterator;
        if (! pattern.empty())
        {
            const GCodePathConfig& config = gcode_layer.configs_storage_.prime_tower_config_per_extruder[extruder_nr];
            gcode_layer.addPolygonsByOptimizer(pattern, config);
            return true;
        }
    }

    return false;
}

bool PrimeTower::addToGcode_inset(LayerPlan& gcode_layer, const size_t extruder_nr) const
{
    const size_t raft_total_extra_layers = Raft::getTotalExtraLayers();
    LayerIndex absolute_layer_number = gcode_layer.getLayerNr() + raft_total_extra_layers;

    if (absolute_layer_number == 0) // Extra-adhesion on very first layer only
    {
        const Shape& pattern_extra_inset = inset_extra_moves_.at(extruder_nr);
        if (! pattern_extra_inset.empty())
        {
            const GCodePathConfig& config = gcode_layer.configs_storage_.prime_tower_config_per_extruder[extruder_nr];
            gcode_layer.addPolygonsByOptimizer(pattern_extra_inset, config);
            return true;
        }
    }

    return false;
}

void PrimeTower::addToGcode_sparseInfill(LayerPlan& gcode_layer, const std::vector<size_t>& extruders_to_prime_idx, const size_t current_extruder_nr) const
{
    std::vector<std::vector<size_t>> extruders_to_prime_idx_grouped;

    // Group extruders which are besides each other
    for (size_t extruder_to_prime_idx : extruders_to_prime_idx)
    {
        if (extruders_to_prime_idx_grouped.empty())
        {
            // First extruder : create new group
            extruders_to_prime_idx_grouped.push_back({ extruder_to_prime_idx });
        }
        else
        {
            std::vector<size_t>& last_group = extruders_to_prime_idx_grouped.back();
            if (last_group.back() == extruder_to_prime_idx - 1)
            {
                // New extruders which belongs to same group
                last_group.push_back(extruder_to_prime_idx);
            }
            else
            {
                // New extruders which belongs to new group
                extruders_to_prime_idx_grouped.push_back({ extruder_to_prime_idx });
            }
        }
    }

    // And finally, append patterns for each group
    const GCodePathConfig& config = gcode_layer.configs_storage_.prime_tower_config_per_extruder[current_extruder_nr];

    for (const std::vector<size_t>& group_idx : extruders_to_prime_idx_grouped)
    {
        size_t mask = 0;
        for (const size_t& extruder_idx : group_idx)
        {
            mask |= (1 << extruder_order_.at(extruder_idx));
        }

        auto iterator_combination = sparse_pattern_per_extruders_.find(mask);
        if (iterator_combination != sparse_pattern_per_extruders_.end())
        {
            const std::map<size_t, Shape>& infill_for_combination = iterator_combination->second;

            auto iterator_extruder_nr = infill_for_combination.find(current_extruder_nr);
            if (iterator_extruder_nr != infill_for_combination.end())
            {
                gcode_layer.addPolygonsByOptimizer(iterator_extruder_nr->second, config);
            }
            else
            {
                spdlog::warn("Sparse pattern not found for extruder {}, skipping\n", current_extruder_nr);
            }
        }
        else
        {
            spdlog::warn("Sparse pattern not found for group {}, skipping\n", mask);
        }
    }
}

void PrimeTower::subtractFromSupport(SliceDataStorage& storage)
{
    for (size_t layer = 0; static_cast<int>(layer) <= storage.max_print_height_second_to_last_extruder + 1 && layer < storage.support.supportLayers.size(); layer++)
    {
        const Shape outside_polygon = getOuterPoly(layer).getOutsidePolygons();
        AABB outside_polygon_boundary_box(outside_polygon);
        SupportLayer& support_layer = storage.support.supportLayers[layer];
        // take the differences of the support infill parts and the prime tower area
        support_layer.excludeAreasFromSupportInfillAreas(outside_polygon, outside_polygon_boundary_box);
    }
}

const Shape& PrimeTower::getOuterPoly(const LayerIndex& layer_nr) const
{
    auto iterator = outer_poly_base_.iterator_at(layer_nr);
    if (iterator != outer_poly_base_.end())
    {
        return *iterator;
    }
    else
    {
        return outer_poly_;
    }
}

const Shape& PrimeTower::getGroundPoly() const
{
    return getOuterPoly(-Raft::getTotalExtraLayers());
}

PrimeTower* PrimeTower::createPrimeTower(SliceDataStorage& storage)
{
    PrimeTower* prime_tower = nullptr;
    const Scene& scene = Application::getInstance().current_slice_->scene;
    const size_t raft_total_extra_layers = Raft::getTotalExtraLayers();

    if (scene.extruders.size() > 1 && scene.current_mesh_group->settings.get<bool>("prime_tower_enable")
        && scene.current_mesh_group->settings.get<coord_t>("prime_tower_min_volume") > 10 && scene.current_mesh_group->settings.get<coord_t>("prime_tower_size") > 10
        && storage.max_print_height_second_to_last_extruder >= -static_cast<int>(raft_total_extra_layers))
    {
        const Settings& mesh_group_settings = scene.current_mesh_group->settings;
        const PrimeTowerMode method = mesh_group_settings.get<PrimeTowerMode>("prime_tower_mode");

        switch (method)
        {
        case PrimeTowerMode::NORMAL:
            prime_tower = new PrimeTowerNormal(storage, scene.extruders.size());
            break;
        case PrimeTowerMode::INTERLEAVED:
            prime_tower = new PrimeTowerInterleaved(storage, scene.extruders.size());
            break;
        }
    }

    if (prime_tower)
    {
        prime_tower->generatePaths();
        prime_tower->subtractFromSupport(storage);
    }

    return prime_tower;
}

bool PrimeTower::extruderRequiresPrime(const std::vector<bool>& extruder_is_used_on_this_layer, size_t extruder_nr, size_t last_extruder)
{
    return extruder_is_used_on_this_layer[extruder_nr] && extruder_nr != last_extruder;
}

void PrimeTower::gotoStartLocation(LayerPlan& gcode_layer, const size_t extruder_nr) const
{
    if (gcode_layer.getLayerNr() != 0)
    {
        size_t current_start_location_idx = ((((extruder_nr + 1) * gcode_layer.getLayerNr()) % number_of_prime_tower_start_locations_) + number_of_prime_tower_start_locations_)
                                          % number_of_prime_tower_start_locations_;

        const ClosestPointPolygon wipe_location = prime_tower_start_locations_[current_start_location_idx];
        const ExtruderTrain& train = Application::getInstance().current_slice_->scene.extruders[extruder_nr];
        const coord_t inward_dist = train.settings_.get<coord_t>("machine_nozzle_size") * 3 / 2;
        const coord_t start_dist = train.settings_.get<coord_t>("machine_nozzle_size") * 2;
        const Point2LL prime_end = PolygonUtils::moveInsideDiagonally(wipe_location, inward_dist);
        const Point2LL outward_dir = wipe_location.location_ - prime_end;
        const Point2LL prime_start = wipe_location.location_ + normal(outward_dir, start_dist);

        gcode_layer.addTravel(prime_start);
    }
}

} // namespace cura
