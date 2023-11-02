// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PrimeTower.h"

#include <algorithm>
#include <limits>

#include <spdlog/spdlog.h>

#include "Application.h" //To get settings.
#include "ExtruderTrain.h"
#include "LayerPlan.h"
#include "PrintFeature.h"
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

PrimeTower::PrimeTower()
    : wipe_from_middle(false)
{
    const Scene& scene = Application::getInstance().current_slice->scene;
    PrimeTowerMethod method = scene.current_mesh_group->settings.get<PrimeTowerMethod>("prime_tower_mode");

    {
        EPlatformAdhesion adhesion_type = scene.current_mesh_group->settings.get<EPlatformAdhesion>("adhesion_type");

        // When we have multiple extruders sharing the same heater/nozzle, we expect that all the extruders have been
        //'primed' by the print-start gcode script, but we don't know which one has been left at the tip of the nozzle
        // and whether it needs 'purging' (before extruding a pure material) or not, so we need to prime (actually purge)
        // each extruder before it is used for the model. This can done by the (per-extruder) brim lines or (per-extruder)
        // skirt lines when they are used, but we need to do that inside the first prime-tower layer when they are not
        // used (sacrifying for this purpose the usual single-extruder first layer, that would be better for prime-tower
        // adhesion).

        multiple_extruders_on_first_layer = (method == PrimeTowerMethod::OPTIMIZED) || (method == PrimeTowerMethod::OPTIMIZED_CONSISTENT)
                                         || (scene.current_mesh_group->settings.get<bool>("machine_extruders_share_nozzle")
                                             && ((adhesion_type != EPlatformAdhesion::SKIRT) && (adhesion_type != EPlatformAdhesion::BRIM)));
    }

    enabled = method != PrimeTowerMethod::NONE && scene.current_mesh_group->settings.get<coord_t>("prime_tower_min_volume") > 10
           && scene.current_mesh_group->settings.get<coord_t>("prime_tower_size") > 10;

    would_have_actual_tower = enabled; // Assume so for now.

    extruder_count = scene.extruders.size();
    extruder_order.resize(extruder_count);
    for (unsigned int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        extruder_order[extruder_nr] = extruder_nr; // Start with default order, then sort.
    }
    // Sort from high adhesion to low adhesion.
    const Scene* scene_pointer = &scene; // Communicate to lambda via pointer to prevent copy.
    std::stable_sort(
        extruder_order.begin(),
        extruder_order.end(),
        [scene_pointer](const unsigned int& extruder_nr_a, const unsigned int& extruder_nr_b) -> bool
        {
            const Ratio adhesion_a = scene_pointer->extruders[extruder_nr_a].settings.get<Ratio>("material_adhesion_tendency");
            const Ratio adhesion_b = scene_pointer->extruders[extruder_nr_b].settings.get<Ratio>("material_adhesion_tendency");
            return adhesion_a < adhesion_b;
        });
#warning TBD take care of actual extruder order for optimized tower !
}

void PrimeTower::checkUsed(const SliceDataStorage& storage)
{
    std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
    size_t used_extruder_count = 0;
    for (bool is_used : extruder_is_used)
    {
        used_extruder_count += is_used;
    }
    if (used_extruder_count <= 1)
    {
        enabled = false;
    }
}

void PrimeTower::generateGroundpoly()
{
    if (! enabled)
    {
        return;
    }

    const Scene& scene = Application::getInstance().current_slice->scene;
    const Settings& mesh_group_settings = scene.current_mesh_group->settings;
    const coord_t tower_size = mesh_group_settings.get<coord_t>("prime_tower_size");

    const coord_t x = mesh_group_settings.get<coord_t>("prime_tower_position_x");
    const coord_t y = mesh_group_settings.get<coord_t>("prime_tower_position_y");
    const coord_t tower_radius = tower_size / 2;
    outer_poly.add(PolygonUtils::makeCircle(Point(x - tower_radius, y + tower_radius), tower_radius, TAU / CIRCLE_RESOLUTION));
    middle = Point(x - tower_size / 2, y + tower_size / 2);

    post_wipe_point = Point(x - tower_size / 2, y + tower_size / 2);
}

void PrimeTower::generatePaths(const SliceDataStorage& storage)
{
    would_have_actual_tower
        = storage.max_print_height_second_to_last_extruder >= 0; // Maybe it turns out that we don't need a prime tower after all because there are no layer switches.
    if (would_have_actual_tower && enabled)
    {
        generateGroundpoly();

        std::vector<coord_t> cumulative_insets;
        generatePaths_denseInfill(cumulative_insets);

        generateStartLocations();

        generatePaths_sparseInfill(cumulative_insets);
    }
}

void PrimeTower::generatePaths_denseInfill(std::vector<coord_t>& cumulative_insets)
{
    const Scene& scene = Application::getInstance().current_slice->scene;
    const Settings& mesh_group_settings = scene.current_mesh_group->settings;
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
    const PrimeTowerMethod method = mesh_group_settings.get<PrimeTowerMethod>("prime_tower_mode");
    const bool base_enabled = mesh_group_settings.get<bool>("prime_tower_brim_enable");
    const coord_t base_extra_radius = scene.settings.get<coord_t>("prime_tower_base_size");
    const bool has_raft = mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT;
    const coord_t base_height = std::max(scene.settings.get<coord_t>("prime_tower_base_height"), has_raft ? layer_height : 0);
    const double base_curve_magnitude = mesh_group_settings.get<double>("prime_tower_base_curve_magnitude");
    const coord_t line_width = scene.extruders[extruder_order.front()].settings.get<coord_t>("prime_tower_line_width");

    prime_moves.resize(extruder_count);
    base_extra_moves.resize(extruder_count);

    coord_t cumulative_inset = 0; // Each tower shape is going to be printed inside the other. This is the inset we're doing for each extruder.
    for (size_t extruder_nr : extruder_order)
    {
        const coord_t line_width = scene.extruders[extruder_nr].settings.get<coord_t>("prime_tower_line_width");
        const coord_t required_volume = MM3_2INT(scene.extruders[extruder_nr].settings.get<double>("prime_tower_min_volume"));
        const Ratio flow = scene.extruders[extruder_nr].settings.get<Ratio>("prime_tower_flow");
        coord_t current_volume = 0;
        Polygons& pattern = prime_moves[extruder_nr];

        // Create the walls of the prime tower.
        unsigned int wall_nr = 0;
        for (; current_volume < required_volume; wall_nr++)
        {
            // Create a new polygon with an offset from the outer polygon.
            Polygons polygons = outer_poly.offset(-cumulative_inset - wall_nr * line_width - line_width / 2);
            pattern.add(polygons);
            current_volume += polygons.polygonLength() * line_width * layer_height * flow;
            if (polygons.empty()) // Don't continue. We won't ever reach the required volume because it doesn't fit.
            {
                break;
            }
        }

        // The most outside extruder is used for the base
        if (extruder_nr == extruder_order.front() && (base_enabled || has_raft) && base_extra_radius > 0 && base_height > 0)
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
                outer_poly_base.push_back(outer_poly.offset(extra_radius));

                base_extra_moves[extruder_nr].push_back(PolygonUtils::generateOutset(outer_poly, extra_rings, line_width));
            }
        }

        cumulative_inset += wall_nr * line_width;
        cumulative_insets.push_back(cumulative_inset);

        // Only the most inside extruder needs to fill the inside of the prime tower
        if (extruder_nr == extruder_order.back())
        {
            Polygons pattern = PolygonUtils::generateInset(outer_poly, line_width, cumulative_inset);
            if (! pattern.empty())
            {
                base_extra_moves[extruder_nr].push_back(pattern);
            }
        }
    }
}

void PrimeTower::generatePaths_sparseInfill(const std::vector<coord_t>& cumulative_insets)
{
    const Scene& scene = Application::getInstance().current_slice->scene;
    const Settings& mesh_group_settings = scene.current_mesh_group->settings;
    const PrimeTowerMethod method = mesh_group_settings.get<PrimeTowerMethod>("prime_tower_mode");

    struct ActualExtruder
    {
        size_t number;
        coord_t line_width;
    };

    std::vector<ActualExtruder> actual_extruders;
    actual_extruders.reserve(extruder_order.size());
    for (size_t extruder_nr : extruder_order)
    {
        const coord_t line_width = scene.extruders[extruder_nr].settings.get<coord_t>("prime_tower_line_width");
        actual_extruders.push_back({ extruder_nr, line_width });
    }

    if (method == PrimeTowerMethod::OPTIMIZED || method == PrimeTowerMethod::OPTIMIZED_CONSISTENT)
    {
        const size_t nb_extruders = scene.extruders.size();

        // Pre-compute radiuses of each extruder ring
        std::vector<coord_t> rings_radii;
        const coord_t tower_size = mesh_group_settings.get<coord_t>("prime_tower_size");
        const coord_t tower_radius = tower_size / 2;

        rings_radii.push_back(tower_radius);
        for (const coord_t& cumulative_inset : cumulative_insets)
        {
            rings_radii.push_back(tower_radius - cumulative_inset);
        }

        // Generate all possible extruders combinations, e.g. if there are 4 extruders, we have combinations
        // 0 / 0-1 / 0-1-2 / 0-1-2-3 / 1 / 1-2 / 1-2-3 / 2 / 2-3 / 3
        // A combination is represented by a bitmask
#warning this should be iterated in outer-to-inner order
        for (size_t first_extruder = 0; first_extruder < nb_extruders; ++first_extruder)
        {
            for (size_t last_extruder = first_extruder; last_extruder < nb_extruders; ++last_extruder)
            {
                size_t extruders_combination = 0;
                for (size_t extruder_nr = first_extruder; extruder_nr <= last_extruder; ++extruder_nr)
                {
                    extruders_combination |= (1 << extruder_nr);
                }

                std::map<size_t, ExtrusionMoves> infills_for_combination;
                for (const ActualExtruder& actual_extruder : actual_extruders)
                {
                    ExtrusionMoves infill = generatePath_sparseInfill(first_extruder, last_extruder, rings_radii, actual_extruder.line_width, actual_extruder.number);
                    infills_for_combination[actual_extruder.number] = infill;
                }

                sparse_pattern_per_extruders[extruders_combination] = infills_for_combination;
            }
        }
    }
}

PrimeTower::ExtrusionMoves PrimeTower::generatePath_sparseInfill(
    const size_t first_extruder,
    const size_t last_extruder,
    const std::vector<coord_t>& rings_radii,
    const coord_t line_width,
    const size_t actual_extruder_nr)
{
    const Scene& scene = Application::getInstance().current_slice->scene;
    const coord_t max_bridging_distance = scene.extruders[actual_extruder_nr].settings.get<coord_t>("prime_tower_max_bridging_distance");
    const coord_t outer_radius = rings_radii[first_extruder];
    const coord_t inner_radius = rings_radii[last_extruder + 1];
    const coord_t radius_delta = outer_radius - inner_radius;
    const coord_t semi_line_width = line_width / 2;

    // Split ring according to max bridging distance
    const size_t nb_rings = std::ceil(static_cast<float>(radius_delta) / max_bridging_distance);
    const coord_t actual_radius_step = radius_delta / nb_rings;

    ExtrusionMoves pattern;
    for (size_t i = 0; i < nb_rings; ++i)
    {
        const coord_t ring_inner_radius = (inner_radius + i * actual_radius_step) + semi_line_width;
        const coord_t ring_outer_radius = (inner_radius + (i + 1) * actual_radius_step) - semi_line_width;

        const size_t semi_nb_spokes = std::ceil((M_PI * ring_outer_radius) / max_bridging_distance);

        pattern.polygons.add(PolygonUtils::makeWheel(middle, ring_inner_radius, ring_outer_radius, semi_nb_spokes, ARC_RESOLUTION));
    }

    return pattern;
}

void PrimeTower::generateStartLocations()
{
    // Evenly spread out a number of dots along the prime tower's outline. This is done for the complete outline,
    // so use the same start and end segments for this.
    PolygonsPointIndex segment_start = PolygonsPointIndex(&outer_poly, 0, 0);
    PolygonsPointIndex segment_end = segment_start;

    PolygonUtils::spreadDots(segment_start, segment_end, number_of_prime_tower_start_locations, prime_tower_start_locations);
}

void PrimeTower::addToGcode(
    const SliceDataStorage& storage,
    LayerPlan& gcode_layer,
    const std::vector<ExtruderUse>& required_extruder_prime,
    const size_t prev_extruder,
    const size_t new_extruder) const
{
    if (! (enabled && would_have_actual_tower))
    {
        return;
    }

    if (gcode_layer.getPrimeTowerIsPlanned(new_extruder))
    { // don't print the prime tower if it has been printed already with this extruder.
        return;
    }

    const LayerIndex layer_nr = gcode_layer.getLayerNr();
    if (layer_nr > storage.max_print_height_second_to_last_extruder + 1)
    {
        return;
    }

    bool post_wipe = Application::getInstance().current_slice->scene.extruders[prev_extruder].settings.get<bool>("prime_tower_wipe_enabled");

    // Do not wipe on the first layer, we will generate non-hollow prime tower there for better bed adhesion.
    if (prev_extruder == new_extruder || layer_nr == 0)
    {
        post_wipe = false;
    }

    // Go to the start location if it's not the first layer
    if (layer_nr != 0)
    {
        gotoStartLocation(gcode_layer, new_extruder);
    }

    PrimeTowerMethod method = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<PrimeTowerMethod>("prime_tower_mode");
    std::vector<size_t> extra_primed_extruders;

    auto extruder_iterator = std::find_if(
        required_extruder_prime.begin(),
        required_extruder_prime.end(),
        [new_extruder](const ExtruderUse& extruder_use)
        {
            return extruder_use.extruder_nr == new_extruder;
        });

    if (extruder_iterator == required_extruder_prime.end())
    {
        // Extruder is not used on this lyer
        return;
    }

    switch (extruder_iterator->prime)
    {
    case ExtruderPrime::None:
        if (method != PrimeTowerMethod::OPTIMIZED)
        {
            gcode_layer.setPrimeTowerIsPlanned(new_extruder);
        }
        break;

    case ExtruderPrime::Sparse:
        extra_primed_extruders = findExtrudersSparseInfill(gcode_layer, required_extruder_prime, new_extruder, method, { new_extruder });
        addToGcode_optimizedInfill(gcode_layer, extra_primed_extruders, new_extruder);
        break;

    case ExtruderPrime::Prime:
        addToGcode_denseInfill(gcode_layer, new_extruder);
        gcode_layer.setPrimeTowerIsPlanned(new_extruder);

        if (method == PrimeTowerMethod::OPTIMIZED && gcode_layer.getLayerNr() < storage.max_print_height_second_to_last_extruder)
        {
            // Whatever happens before and after, use the current extruder to prime all the non-required extruders now
            extra_primed_extruders = findExtrudersSparseInfill(gcode_layer, required_extruder_prime, new_extruder, method);
            addToGcode_optimizedInfill(gcode_layer, extra_primed_extruders, new_extruder);
        }
        break;
    }

    for (const size_t& primed_extruder : extra_primed_extruders)
    {
        gcode_layer.setPrimeTowerIsPlanned(primed_extruder);
    }

    // post-wipe:
    if (post_wipe)
    {
        // Make sure we wipe the old extruder on the prime tower.
        const Settings& previous_settings = Application::getInstance().current_slice->scene.extruders[prev_extruder].settings;
        const Point previous_nozzle_offset = Point(previous_settings.get<coord_t>("machine_nozzle_offset_x"), previous_settings.get<coord_t>("machine_nozzle_offset_y"));
        const Settings& new_settings = Application::getInstance().current_slice->scene.extruders[new_extruder].settings;
        const Point new_nozzle_offset = Point(new_settings.get<coord_t>("machine_nozzle_offset_x"), new_settings.get<coord_t>("machine_nozzle_offset_y"));
        gcode_layer.addTravel(post_wipe_point - previous_nozzle_offset + new_nozzle_offset);
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
        const GCodePathConfig& config = gcode_layer.configs_storage.prime_tower_config_per_extruder[extruder_nr];
        const Polygons& pattern = prime_moves[extruder_nr];
        gcode_layer.addPolygonsByOptimizer(pattern, config);
    }

    const std::vector<Polygons>& pattern_extra_brim = base_extra_moves[extruder_nr];
    if (absolute_layer_number < pattern_extra_brim.size())
    {
        // Extra rings for stronger base
        const GCodePathConfig& config = gcode_layer.configs_storage.prime_tower_config_per_extruder[extruder_nr];
        const Polygons& pattern = pattern_extra_brim[absolute_layer_number];
        gcode_layer.addPolygonsByOptimizer(pattern, config);
    }
}

void PrimeTower::addToGcode_optimizedInfill(LayerPlan& gcode_layer, const std::vector<size_t>& extruders_to_prime, const size_t current_extruder) const
{
    std::vector<std::vector<size_t>> extruders_to_prime_grouped;

    // Group extruders which are besides each other
    for (size_t extruder_to_prime : extruders_to_prime)
    {
        if (extruders_to_prime_grouped.empty())
        {
            // First extruder : create new group
            extruders_to_prime_grouped.push_back({ extruder_to_prime });
        }
        else
        {
            std::vector<size_t>& last_group = extruders_to_prime_grouped.back();
#warning This does not take care of extruders ordering
            if (last_group.back() == extruder_to_prime - 1)
            {
                // New extruders which belongs to same group
                last_group.push_back(extruder_to_prime);
            }
            else
            {
                // New extruders which belongs to new group
                extruders_to_prime_grouped.push_back({ extruder_to_prime });
            }
        }
    }

    std::string grouped_extruders;
    for (auto& group : extruders_to_prime_grouped)
    {
        grouped_extruders.append("{");
        for (auto& extruder : group)
        {
            grouped_extruders.append(std::to_string(extruder));
            grouped_extruders.append(",");
        }
        grouped_extruders.append("}");
    }

#warning What should we do in case of extruders with different lines widths ?
    // And finally, append patterns for each group
    const GCodePathConfig& config = gcode_layer.configs_storage.prime_tower_config_per_extruder[current_extruder];

    for (const std::vector<size_t>& group : extruders_to_prime_grouped)
    {
        size_t mask = 0;
        for (const size_t& extruder : group)
        {
            mask |= (1 << extruder);
        }

        auto iterator_combination = sparse_pattern_per_extruders.find(mask);
        if (iterator_combination != sparse_pattern_per_extruders.end())
        {
            const std::map<size_t, ExtrusionMoves>& infill_for_combination = iterator_combination->second;

            auto iterator_extruder_nr = infill_for_combination.find(current_extruder);
            if (iterator_extruder_nr != infill_for_combination.end())
            {
                gcode_layer.addPolygonsByOptimizer(iterator_extruder_nr->second.polygons, config);
            }
            else
            {
                spdlog::warn("Sparse pattern not found for extruder {}, skipping\n", current_extruder);
            }
        }
        else
        {
            spdlog::warn("Sparse pattern not found for group {}, skipping\n", mask);
        }
    }
}

std::vector<size_t> PrimeTower::findExtrudersSparseInfill(
    LayerPlan& gcode_layer,
    const std::vector<ExtruderUse>& required_extruder_prime,
    const size_t current_extruder,
    PrimeTowerMethod method,
    const std::vector<size_t>& initial_list) const
{
    std::vector<size_t> extruders_to_prime;

    for (size_t extruder_nr : extruder_order)
    {
        auto iterator_initial_list = std::find(initial_list.begin(), initial_list.end(), extruder_nr);
        bool is_in_initial_list = iterator_initial_list != initial_list.end();

        if (is_in_initial_list)
        {
            extruders_to_prime.push_back(extruder_nr);
        }
        else if (method == PrimeTowerMethod::OPTIMIZED && ! gcode_layer.getPrimeTowerIsPlanned(extruder_nr))
        {
            auto iterator_required_list = std::find_if(
                required_extruder_prime.begin(),
                required_extruder_prime.end(),
                [extruder_nr](const ExtruderUse& extruder_use)
                {
                    return extruder_use.extruder_nr == extruder_nr && extruder_use.prime == ExtruderPrime::Prime;
                });
            bool is_in_required_list = iterator_required_list != required_extruder_prime.end();

            if (! is_in_required_list)
            {
                extruders_to_prime.push_back(extruder_nr);
            }
        }
    }

    return extruders_to_prime;
}

void PrimeTower::subtractFromSupport(SliceDataStorage& storage)
{
    for (size_t layer = 0; layer <= (size_t)storage.max_print_height_second_to_last_extruder + 1 && layer < storage.support.supportLayers.size(); layer++)
    {
        const Polygons outside_polygon = getOuterPoly(layer).getOutsidePolygons();
        AABB outside_polygon_boundary_box(outside_polygon);
        SupportLayer& support_layer = storage.support.supportLayers[layer];
        // take the differences of the support infill parts and the prime tower area
        support_layer.excludeAreasFromSupportInfillAreas(outside_polygon, outside_polygon_boundary_box);
    }
}

const Polygons& PrimeTower::getOuterPoly(const LayerIndex& layer_nr) const
{
    const LayerIndex absolute_layer_nr = layer_nr + Raft::getTotalExtraLayers();
    if (absolute_layer_nr < outer_poly_base.size())
    {
        return outer_poly_base[absolute_layer_nr];
    }
    else
    {
        return outer_poly;
    }
}

const Polygons& PrimeTower::getGroundPoly() const
{
    return getOuterPoly(-Raft::getTotalExtraLayers());
}

void PrimeTower::gotoStartLocation(LayerPlan& gcode_layer, const int extruder_nr) const
{
    int current_start_location_idx = ((((extruder_nr + 1) * gcode_layer.getLayerNr()) % number_of_prime_tower_start_locations) + number_of_prime_tower_start_locations)
                                   % number_of_prime_tower_start_locations;

    const ClosestPolygonPoint wipe_location = prime_tower_start_locations[current_start_location_idx];

    const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];
    const coord_t inward_dist = train.settings.get<coord_t>("machine_nozzle_size") * 3 / 2;
    const coord_t start_dist = train.settings.get<coord_t>("machine_nozzle_size") * 2;
    const Point prime_end = PolygonUtils::moveInsideDiagonally(wipe_location, inward_dist);
    const Point outward_dir = wipe_location.location - prime_end;
    const Point prime_start = wipe_location.location + normal(outward_dir, start_dist);

    gcode_layer.addTravel(prime_start);
}

} // namespace cura
