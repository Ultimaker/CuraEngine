// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PrimeTower.h"

#include <algorithm>
#include <limits>

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


namespace cura
{

PrimeTower::PrimeTower()
    : wipe_from_middle_(false)
{
    const Scene& scene = Application::getInstance().current_slice_->scene;

    {
        EPlatformAdhesion adhesion_type = scene.current_mesh_group->settings.get<EPlatformAdhesion>("adhesion_type");

        // When we have multiple extruders sharing the same heater/nozzle, we expect that all the extruders have been
        //'primed' by the print-start gcode script, but we don't know which one has been left at the tip of the nozzle
        // and whether it needs 'purging' (before extruding a pure material) or not, so we need to prime (actually purge)
        // each extruder before it is used for the model. This can done by the (per-extruder) brim lines or (per-extruder)
        // skirt lines when they are used, but we need to do that inside the first prime-tower layer when they are not
        // used (sacrifying for this purpose the usual single-extruder first layer, that would be better for prime-tower
        // adhesion).

        multiple_extruders_on_first_layer_ = scene.current_mesh_group->settings.get<bool>("machine_extruders_share_nozzle")
                                          && ((adhesion_type != EPlatformAdhesion::SKIRT) && (adhesion_type != EPlatformAdhesion::BRIM));
    }

    enabled_ = scene.current_mesh_group->settings.get<bool>("prime_tower_enable") && scene.current_mesh_group->settings.get<coord_t>("prime_tower_min_volume") > 10
            && scene.current_mesh_group->settings.get<coord_t>("prime_tower_size") > 10;
    would_have_actual_tower_ = enabled_; // Assume so for now.

    extruder_count_ = scene.extruders.size();
    extruder_order_.resize(extruder_count_);
    for (unsigned int extruder_nr = 0; extruder_nr < extruder_count_; extruder_nr++)
    {
        extruder_order_[extruder_nr] = extruder_nr; // Start with default order, then sort.
    }
    // Sort from high adhesion to low adhesion.
    const Scene* scene_pointer = &scene; // Communicate to lambda via pointer to prevent copy.
    std::stable_sort(
        extruder_order_.begin(),
        extruder_order_.end(),
        [scene_pointer](const unsigned int& extruder_nr_a, const unsigned int& extruder_nr_b) -> bool
        {
            const Ratio adhesion_a = scene_pointer->extruders[extruder_nr_a].settings_.get<Ratio>("material_adhesion_tendency");
            const Ratio adhesion_b = scene_pointer->extruders[extruder_nr_b].settings_.get<Ratio>("material_adhesion_tendency");
            return adhesion_a < adhesion_b;
        });
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
        enabled_ = false;
    }
}

void PrimeTower::generateGroundpoly()
{
    if (! enabled_)
    {
        return;
    }

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

void PrimeTower::generatePaths(const SliceDataStorage& storage)
{
    const int raft_total_extra_layers = Raft::getTotalExtraLayers();
    would_have_actual_tower_ = storage.max_print_height_second_to_last_extruder
                            >= -raft_total_extra_layers; // Maybe it turns out that we don't need a prime tower after all because there are no layer switches.
    if (would_have_actual_tower_ && enabled_)
    {
        generatePaths_denseInfill();
        generateStartLocations();
    }
}

void PrimeTower::generatePaths_denseInfill()
{
    const Scene& scene = Application::getInstance().current_slice_->scene;
    const Settings& mesh_group_settings = scene.current_mesh_group->settings;
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
    const bool base_enabled = mesh_group_settings.get<bool>("prime_tower_brim_enable");
    const coord_t base_extra_radius = scene.settings.get<coord_t>("prime_tower_base_size");
    const bool has_raft = mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT;
    const coord_t base_height = std::max(scene.settings.get<coord_t>("prime_tower_base_height"), has_raft ? layer_height : 0);
    const double base_curve_magnitude = mesh_group_settings.get<double>("prime_tower_base_curve_magnitude");

    prime_moves_.resize(extruder_count_);
    base_extra_moves_.resize(extruder_count_);

    coord_t cumulative_inset = 0; // Each tower shape is going to be printed inside the other. This is the inset we're doing for each extruder.
    for (size_t extruder_nr : extruder_order_)
    {
        const coord_t line_width = scene.extruders[extruder_nr].settings_.get<coord_t>("prime_tower_line_width");
        const coord_t required_volume = MM3_2INT(scene.extruders[extruder_nr].settings_.get<double>("prime_tower_min_volume"));
        const Ratio flow = scene.extruders[extruder_nr].settings_.get<Ratio>("prime_tower_flow");
        coord_t current_volume = 0;
        Polygons& prime_moves = prime_moves_[extruder_nr];

        // Create the walls of the prime tower.
        unsigned int wall_nr = 0;
        for (; current_volume < required_volume; wall_nr++)
        {
            // Create a new polygon with an offset from the outer polygon.
            Polygons polygons = outer_poly_.offset(-cumulative_inset - wall_nr * line_width - line_width / 2);
            prime_moves.add(polygons);
            current_volume += polygons.length() * line_width * layer_height * flow;
            if (polygons.empty()) // Don't continue. We won't ever reach the required volume because it doesn't fit.
            {
                break;
            }
        }

        // The most outside extruder is used for the base
        if (extruder_nr == extruder_order_.front() && (base_enabled || has_raft) && base_extra_radius > 0 && base_height > 0)
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

        // Only the most inside extruder needs to fill the inside of the prime tower
        if (extruder_nr == extruder_order_.back())
        {
            Polygons base_extra_moves = PolygonUtils::generateInset(outer_poly_, line_width, cumulative_inset);
            if (! base_extra_moves.empty())
            {
                base_extra_moves_[extruder_nr].push_back(base_extra_moves);
            }
        }
    }
}

void PrimeTower::generateStartLocations()
{
    // Evenly spread out a number of dots along the prime tower's outline. This is done for the complete outline,
    // so use the same start and end segments for this.
    PolygonsPointIndex segment_start = PolygonsPointIndex(&outer_poly_, 0, 0);
    PolygonsPointIndex segment_end = segment_start;

    PolygonUtils::spreadDots(segment_start, segment_end, number_of_prime_tower_start_locations_, prime_tower_start_locations_);
}

void PrimeTower::addToGcode(const SliceDataStorage& storage, LayerPlan& gcode_layer, const size_t prev_extruder, const size_t new_extruder) const
{
    if (! (enabled_ && would_have_actual_tower_))
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

    bool post_wipe = Application::getInstance().current_slice_->scene.extruders[prev_extruder].settings_.get<bool>("prime_tower_wipe_enabled");

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

    addToGcode_denseInfill(gcode_layer, new_extruder);

    // post-wipe:
    if (post_wipe)
    {
        // Make sure we wipe the old extruder on the prime tower.
        const Settings& previous_settings = Application::getInstance().current_slice_->scene.extruders[prev_extruder].settings_;
        const Point2LL previous_nozzle_offset = Point2LL(previous_settings.get<coord_t>("machine_nozzle_offset_x"), previous_settings.get<coord_t>("machine_nozzle_offset_y"));
        const Settings& new_settings = Application::getInstance().current_slice_->scene.extruders[new_extruder].settings_;
        const Point2LL new_nozzle_offset = Point2LL(new_settings.get<coord_t>("machine_nozzle_offset_x"), new_settings.get<coord_t>("machine_nozzle_offset_y"));
        gcode_layer.addTravel(post_wipe_point_ - previous_nozzle_offset + new_nozzle_offset);
    }

    gcode_layer.setPrimeTowerIsPlanned(new_extruder);
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
        const Polygons& pattern = prime_moves_[extruder_nr];
        gcode_layer.addPolygonsByOptimizer(pattern, config);
    }

    const std::vector<Polygons>& pattern_extra_brim = base_extra_moves_[extruder_nr];
    if (absolute_layer_number < pattern_extra_brim.size())
    {
        // Extra rings for stronger base
        const GCodePathConfig& config = gcode_layer.configs_storage_.prime_tower_config_per_extruder[extruder_nr];
        const Polygons& pattern = pattern_extra_brim[absolute_layer_number];
        gcode_layer.addPolygonsByOptimizer(pattern, config);
    }
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
    if (absolute_layer_nr < outer_poly_base_.size())
    {
        return outer_poly_base_[absolute_layer_nr];
    }
    else
    {
        return outer_poly_;
    }
}

const Polygons& PrimeTower::getGroundPoly() const
{
    return getOuterPoly(-Raft::getTotalExtraLayers());
}

void PrimeTower::gotoStartLocation(LayerPlan& gcode_layer, const int extruder_nr) const
{
    int current_start_location_idx = ((((extruder_nr + 1) * gcode_layer.getLayerNr()) % number_of_prime_tower_start_locations_) + number_of_prime_tower_start_locations_)
                                   % number_of_prime_tower_start_locations_;

    const ClosestPoint wipe_location = prime_tower_start_locations_[current_start_location_idx];

    const ExtruderTrain& train = Application::getInstance().current_slice_->scene.extruders[extruder_nr];
    const coord_t inward_dist = train.settings_.get<coord_t>("machine_nozzle_size") * 3 / 2;
    const coord_t start_dist = train.settings_.get<coord_t>("machine_nozzle_size") * 2;
    const Point2LL prime_end = PolygonUtils::moveInsideDiagonally(wipe_location, inward_dist);
    const Point2LL outward_dir = wipe_location.location_ - prime_end;
    const Point2LL prime_start = wipe_location.location_ + normal(outward_dir, start_dist);

    gcode_layer.addTravel(prime_start);
}

} // namespace cura
