// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "prime_tower/PrimeTower.h"

#include <algorithm>
#include <limits>
#include <numbers>

#include <spdlog/spdlog.h>

#include "Application.h" //To get settings.
#include "ExtruderTrain.h"
#include "LayerPlan.h"
#include "prime_tower/PrimeTowerInterleaved.h"
#include "prime_tower/PrimeTowerNormal.h"
#include "Raft.h"
#include "Scene.h"
#include "Slice.h"
#include "SliceDataStorage.h"
#include "SupportLayer.h"
#include "gcode_export/GCodeExport.h"
#include "infill/Infill.h"
#include "types/PrimeTowerMode.h"
#include "utils/LinearAlg2D.h"


namespace curaengine::prime_tower
{

PrimeTower::PrimeTower()
    : wipe_from_middle_(false)
{
    const Scene& scene = Application::getInstance().current_slice_->scene;
    const Settings& mesh_group_settings = scene.current_mesh_group->settings;
    const ::curaengine::utils::coord_t tower_radius = mesh_group_settings.get<::curaengine::utils::coord_t>("prime_tower_size") / 2;
    const ::curaengine::utils::coord_t x = mesh_group_settings.get<::curaengine::utils::coord_t>("prime_tower_position_x");
    const ::curaengine::utils::coord_t y = mesh_group_settings.get<::curaengine::utils::coord_t>("prime_tower_position_y");
    const ::curaengine::utils::coord_t layer_height = mesh_group_settings.get<::curaengine::utils::coord_t>("layer_height");
    const bool base_enabled = mesh_group_settings.get<bool>("prime_tower_brim_enable");
    const ::curaengine::utils::coord_t base_extra_radius = scene.settings.get<::curaengine::utils::coord_t>("prime_tower_base_size");
    const ::curaengine::utils::coord_t base_height = scene.settings.get<::curaengine::utils::coord_t>("prime_tower_base_height");
    const double base_curve_magnitude = mesh_group_settings.get<double>("prime_tower_base_curve_magnitude");

    middle_ = ::curaengine::geometry::Point2LL(x - tower_radius, y + tower_radius);
    outer_poly_ = { ::curaengine::utils::PolygonUtils::makeDisc(middle_, tower_radius, circle_definition_), tower_radius };
    post_wipe_point_ = ::curaengine::geometry::Point2LL(x - tower_radius, y + tower_radius);

    // Generate the base outline
    if (base_enabled && base_extra_radius > 0 && base_height > 0)
    {
        base_occupied_outline_.init(true);

        for (::curaengine::utils::coord_t z = 0; z < base_height; z += layer_height)
        {
            const ::curaengine::utils::coord_t extra_radius = ::curaengine::utils::LinearAlg2D::getSlopedWidth(base_extra_radius, base_height, base_curve_magnitude, z);
            const ::curaengine::utils::coord_t total_radius = tower_radius + extra_radius;
            base_occupied_outline_.push_back(OccupiedOutline{ ::curaengine::utils::PolygonUtils::makeDisc(middle_, total_radius, circle_definition_), total_radius });
        }
    }
}

void PrimeTower::generateBase()
{
    const Scene& scene = Application::getInstance().current_slice_->scene;
    const Settings& mesh_group_settings = scene.current_mesh_group->settings;
    const bool base_enabled = mesh_group_settings.get<bool>("prime_tower_brim_enable");
    const ::curaengine::utils::coord_t base_extra_radius = scene.settings.get<::curaengine::utils::coord_t>("prime_tower_base_size");
    const ::curaengine::utils::coord_t base_height = scene.settings.get<::curaengine::utils::coord_t>("prime_tower_base_height");

    if (base_enabled && base_extra_radius > 0 && base_height > 0)
    {
        base_extrusion_outline_.init(true);

        // Generate the base outside extra annuli for the first extruder of each layer
        auto iterator_extrusion_paths = toolpaths_.begin();
        auto iterator_base_outline = base_occupied_outline_.begin();
        for (; iterator_extrusion_paths != toolpaths_.end() && iterator_base_outline != base_occupied_outline_.end(); ++iterator_extrusion_paths, ++iterator_base_outline)
        {
            std::vector<ExtruderToolPaths>& toolpaths_at_this_layer = iterator_extrusion_paths->second;
            if (! toolpaths_at_this_layer.empty())
            {
                const OccupiedOutline& base_ouline_at_this_layer = *iterator_base_outline;
                ExtruderToolPaths& first_extruder_toolpaths = toolpaths_at_this_layer.front();
                const size_t extruder_nr = first_extruder_toolpaths.extruder_nr;
                const ::curaengine::utils::coord_t line_width = scene.extruders[extruder_nr].settings_.get<::curaengine::utils::coord_t>("prime_tower_line_width");

                std::tuple<::curaengine::geometry::ClosedLinesSet, ::curaengine::utils::coord_t> outset
                    = ::curaengine::utils::PolygonUtils::generateCirculatOutset(middle_, first_extruder_toolpaths.outer_radius, base_ouline_at_this_layer.outer_radius, line_width, circle_definition_);
                first_extruder_toolpaths.toolpaths.push_back(std::get<0>(outset));

                base_extrusion_outline_.push_back(::curaengine::utils::PolygonUtils::makeDisc(middle_, std::get<1>(outset), circle_definition_));
            }
        }
    }
}

void PrimeTower::generateFirtLayerInset()
{
    // Generate the base inside extra disc for the last extruder of the first layer
    if (! toolpaths_.empty())
    {
        std::vector<ExtruderToolPaths>& toolpaths_first_layer = toolpaths_.begin()->second;
        if (! toolpaths_first_layer.empty())
        {
            ExtruderToolPaths& last_extruder_toolpaths = toolpaths_first_layer.back();
            const Scene& scene = Application::getInstance().current_slice_->scene;
            const size_t extruder_nr = last_extruder_toolpaths.extruder_nr;
            const ::curaengine::utils::coord_t line_width = scene.extruders[extruder_nr].settings_.get<::curaengine::utils::coord_t>("prime_tower_line_width");
            ::curaengine::geometry::ClosedLinesSet pattern = ::curaengine::utils::PolygonUtils::generateCircularInset(middle_, last_extruder_toolpaths.inner_radius, line_width, circle_definition_);
            last_extruder_toolpaths.toolpaths.push_back(pattern);
        }
    }
}

std::tuple<::curaengine::geometry::ClosedLinesSet, ::curaengine::utils::coord_t> PrimeTower::generatePrimeToolpaths(const size_t extruder_nr, const ::curaengine::utils::coord_t outer_radius)
{
    const Scene& scene = Application::getInstance().current_slice_->scene;
    const Settings& mesh_group_settings = scene.current_mesh_group->settings;
    const ::curaengine::utils::coord_t layer_height = mesh_group_settings.get<::curaengine::utils::coord_t>("layer_height");
    const ::curaengine::utils::coord_t line_width = scene.extruders[extruder_nr].settings_.get<::curaengine::utils::coord_t>("prime_tower_line_width");
    const double required_volume = scene.extruders[extruder_nr].settings_.get<double>("prime_tower_min_volume") * 1000000000;
    const ::curaengine::types::Ratio flow = scene.extruders[extruder_nr].settings_.get<::curaengine::types::Ratio>("prime_tower_flow");
    const ::curaengine::utils::coord_t semi_line_width = line_width / 2;

    double current_volume = 0;
    ::curaengine::utils::coord_t current_outer_radius = outer_radius - semi_line_width;
    ::curaengine::geometry::ClosedLinesSet toolpaths;
    while (current_volume < required_volume && current_outer_radius >= semi_line_width)
    {
        ::curaengine::geometry::ClosedPolyline circle = ::curaengine::utils::PolygonUtils::makeCircle(middle_, current_outer_radius, circle_definition_);
        toolpaths.push_back(circle);
        current_volume += static_cast<double>(circle.length() * line_width * layer_height) * flow;
        current_outer_radius -= line_width;
    }

    return { toolpaths, current_outer_radius + semi_line_width };
}

::curaengine::geometry::ClosedLinesSet PrimeTower::generateSupportToolpaths(const size_t extruder_nr, const ::curaengine::utils::coord_t outer_radius, const ::curaengine::utils::coord_t inner_radius)
{
    const Scene& scene = Application::getInstance().current_slice_->scene;
    const Settings& extruder_settings = scene.extruders[extruder_nr].settings_;
    const ::curaengine::utils::coord_t line_width = extruder_settings.get<::curaengine::utils::coord_t>("prime_tower_line_width");
    const double max_bridging_distance = static_cast<double>(std::max(extruder_settings.get<::curaengine::utils::coord_t>("prime_tower_max_bridging_distance"), line_width));
    const ::curaengine::utils::coord_t radius_delta = outer_radius - inner_radius;
    const ::curaengine::utils::coord_t semi_line_width = line_width / 2;

    ::curaengine::geometry::ClosedLinesSet toolpaths;
    // Split annuli according to max bridging distance
    const ::curaengine::utils::coord_t nb_annuli = static_cast<::curaengine::utils::coord_t>(std::ceil(static_cast<double>(radius_delta) / max_bridging_distance));
    const ::curaengine::utils::coord_t nb_circles = radius_delta / line_width;

    if (nb_circles <= nb_annuli)
    {
        // Bridging distance is too small for a proper wheel pattern; use solid concentric circles instead.
        for (::curaengine::utils::coord_t i = 0; i < nb_circles; ++i)
        {
            toolpaths.push_back(::curaengine::utils::PolygonUtils::makeCircle(middle_, outer_radius - semi_line_width - i * line_width, circle_definition_));
        }
    }
    else if (nb_annuli > 0)
    {
        const ::curaengine::utils::coord_t actual_radius_step = radius_delta / nb_annuli;

        for (::curaengine::utils::coord_t i = 0; i < nb_annuli; ++i)
        {
            const ::curaengine::utils::coord_t annulus_inner_radius = (inner_radius + i * actual_radius_step) + semi_line_width;
            const ::curaengine::utils::coord_t annulus_outer_radius = (inner_radius + (i + 1) * actual_radius_step) - semi_line_width;

            const size_t semi_nb_spokes = static_cast<size_t>(std::ceil((std::numbers::pi * static_cast<double>(annulus_outer_radius)) / max_bridging_distance));

            toolpaths.push_back(::curaengine::utils::PolygonUtils::makeWheel(middle_, annulus_inner_radius, annulus_outer_radius, semi_nb_spokes, arc_definition_));
        }
    }

    return toolpaths;
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

    const ::curaengine::types::LayerIndex layer_nr = gcode_layer.getLayerNr();
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

    const ::curaengine::geometry::ClosedLinesSet* toolpaths = nullptr;
    auto iterator_layer = toolpaths_.find(layer_nr);
    if (iterator_layer != toolpaths_.end())
    {
        const std::vector<ExtruderToolPaths>& toolpaths_at_this_layer = iterator_layer->second;
        auto iterator_extruder = std::find_if(
            toolpaths_at_this_layer.begin(),
            toolpaths_at_this_layer.end(),
            [new_extruder_nr](const ExtruderToolPaths& extruder_toolpaths)
            {
                return extruder_toolpaths.extruder_nr == new_extruder_nr;
            });
        if (iterator_extruder != iterator_layer->second.end())
        {
            toolpaths = &(iterator_extruder->toolpaths);
        }
    }

    if (toolpaths && ! toolpaths->empty())
    {
        gotoStartLocation(gcode_layer, new_extruder_nr);

        const GCodePathConfig& config = gcode_layer.configs_storage_.prime_tower_config_per_extruder[new_extruder_nr];
        gcode_layer.addLinesByOptimizer(*toolpaths, config, SpaceFillType::PolyLines);
    }

    gcode_layer.setPrimeTowerIsPlanned(new_extruder_nr);

    // post-wipe:
    if (post_wipe)
    {
        // Make sure we wipe the old extruder on the prime tower.
        const Settings& previous_settings = Application::getInstance().current_slice_->scene.extruders[prev_extruder_nr].settings_;
        const ::curaengine::geometry::Point2LL previous_nozzle_offset = ::curaengine::geometry::Point2LL(previous_settings.get<::curaengine::utils::coord_t>("machine_nozzle_offset_x"), previous_settings.get<::curaengine::utils::coord_t>("machine_nozzle_offset_y"));
        const Settings& new_settings = Application::getInstance().current_slice_->scene.extruders[new_extruder_nr].settings_;
        const ::curaengine::geometry::Point2LL new_nozzle_offset = ::curaengine::geometry::Point2LL(new_settings.get<::curaengine::utils::coord_t>("machine_nozzle_offset_x"), new_settings.get<::curaengine::utils::coord_t>("machine_nozzle_offset_y"));
        gcode_layer.addTravel(post_wipe_point_ - previous_nozzle_offset + new_nozzle_offset);
    }
}

const ::curaengine::geometry::Polygon& PrimeTower::getOccupiedOutline(const ::curaengine::types::LayerIndex& layer_nr) const
{
    auto iterator = base_occupied_outline_.iterator_at(layer_nr);
    if (iterator != base_occupied_outline_.end())
    {
        return iterator->outline;
    }
    else
    {
        return outer_poly_.outline;
    }
}

const ::curaengine::geometry::Polygon& PrimeTower::getOccupiedGroundOutline() const
{
    if (! base_extrusion_outline_.empty())
    {
        return base_extrusion_outline_.front();
    }
    else
    {
        return outer_poly_.outline;
    }
}

const ::curaengine::geometry::Polygon& PrimeTower::getExtrusionOutline(const ::curaengine::types::LayerIndex& layer_nr) const
{
    auto iterator = base_extrusion_outline_.iterator_at(layer_nr);
    if (iterator != base_extrusion_outline_.end())
    {
        return *iterator;
    }
    else
    {
        return outer_poly_.outline;
    }
}

void PrimeTower::subtractFromSupport(SliceDataStorage& storage)
{
    for (size_t layer = 0; static_cast<int>(layer) <= storage.max_print_height_second_to_last_extruder + 1 && layer < storage.support.supportLayers.size(); layer++)
    {
        const ::curaengine::geometry::Polygon& outside_polygon = getOccupiedOutline(layer);
        ::curaengine::utils::AABB outside_polygon_boundary_box(outside_polygon);
        SupportLayer& support_layer = storage.support.supportLayers[layer];
        // take the differences of the support infill parts and the prime tower area
        support_layer.excludeAreasFromSupportInfillAreas(::curaengine::geometry::Shape(outside_polygon), outside_polygon_boundary_box);
    }
}

void PrimeTower::processExtrudersUse(::curaengine::utils::LayerVector<std::vector<ExtruderUse>>& extruders_use, const size_t start_extruder)
{
    polishExtrudersUses(extruders_use, start_extruder);
    toolpaths_ = generateToolPaths(extruders_use);
    generateBase();
    generateFirtLayerInset();
}

PrimeTower* PrimeTower::createPrimeTower(SliceDataStorage& storage)
{
    PrimeTower* prime_tower = nullptr;
    const Settings& settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    const size_t raft_total_extra_layers = Raft::getTotalExtraLayers();
    const std::vector<bool> extruders_used = storage.getExtrudersUsed();

    std::vector<size_t> used_extruders_nrs;
    for (size_t extruder_nr = 0; extruder_nr < extruders_used.size(); extruder_nr++)
    {
        if (extruders_used[extruder_nr])
        {
            used_extruders_nrs.push_back(extruder_nr);
        }
    }

    if (used_extruders_nrs.size() > 1 && settings.get<bool>("prime_tower_enable") && settings.get<::curaengine::utils::coord_t>("prime_tower_min_volume") > 10
        && settings.get<::curaengine::utils::coord_t>("prime_tower_size") > 10 && storage.max_print_height_second_to_last_extruder >= -static_cast<int>(raft_total_extra_layers))
    {
        const ::curaengine::types::PrimeTowerMode method = settings.get<::curaengine::types::PrimeTowerMode>("prime_tower_mode");

        switch (method)
        {
        case ::curaengine::types::PrimeTowerMode::NORMAL:
            prime_tower = new PrimeTowerNormal(used_extruders_nrs);
            break;
        case ::curaengine::types::PrimeTowerMode::INTERLEAVED:
            prime_tower = new PrimeTowerInterleaved();
            break;
        }
    }

    if (prime_tower)
    {
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
    ::curaengine::types::LayerIndex layer_nr = gcode_layer.getLayerNr();
    if (layer_nr != -::curaengine::types::LayerIndex(Raft::getTotalExtraLayers()))
    {
        ::curaengine::utils::coord_t wipe_radius;
        auto iterator = base_occupied_outline_.iterator_at(gcode_layer.getLayerNr());
        if (iterator != base_occupied_outline_.end())
        {
            wipe_radius = iterator->outer_radius;
        }
        else
        {
            wipe_radius = outer_poly_.outer_radius;
        }

        const ExtruderTrain& train = Application::getInstance().current_slice_->scene.extruders[extruder_nr];
        wipe_radius += train.settings_.get<::curaengine::utils::coord_t>("machine_nozzle_size") * 2;

        // Layer number may be negative, make it positive (or null) before using modulo operator
        while (layer_nr < 0)
        {
            layer_nr += number_of_prime_tower_start_locations_;
        }

        size_t current_start_location_idx = ((extruder_nr + 1) * static_cast<size_t>(layer_nr)) % number_of_prime_tower_start_locations_;
        const ::curaengine::types::AngleRadians angle = start_locations_step_ * current_start_location_idx;
        const ::curaengine::geometry::Point2LL prime_start = ::curaengine::utils::PolygonUtils::makeCirclePoint(middle_, wipe_radius, angle);

        gcode_layer.addTravel(prime_start);
    }
}

} // namespace curaengine::prime_tower
