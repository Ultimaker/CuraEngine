// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "InsetOrderOptimizer.h"
#include "ExtruderTrain.h"
#include "FffGcodeWriter.h"
#include "LayerPlan.h"
#include "utils/AABB.h"
#include "utils/SparseLineGrid.h"

#include <iterator>
#include <tuple>
#include <unordered_map>

#include <range/v3/algorithm/max.hpp>
#include <range/v3/to_container.hpp>
#include <range/v3/view/any_view.hpp>
#include <range/v3/view/drop_last.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/remove_if.hpp>
#include <range/v3/view/reverse.hpp>
#include <range/v3/view/take_exactly.hpp>
#include <range/v3/view/transform.hpp>


#include <range/v3/all.hpp>  // TODO: only include what I use
#include <spdlog/spdlog.h>
#include <fmt/ranges.h>

namespace cura
{

InsetOrderOptimizer::InsetOrderOptimizer(const FffGcodeWriter& gcode_writer,
                                         const SliceDataStorage& storage,
                                         LayerPlan& gcode_layer,
                                         const Settings& settings,
                                         const int extruder_nr,
                                         const GCodePathConfig& inset_0_non_bridge_config,
                                         const GCodePathConfig& inset_X_non_bridge_config,
                                         const GCodePathConfig& inset_0_bridge_config,
                                         const GCodePathConfig& inset_X_bridge_config,
                                         const bool retract_before_outer_wall,
                                         const coord_t wall_0_wipe_dist,
                                         const coord_t wall_x_wipe_dist,
                                         const size_t wall_0_extruder_nr,
                                         const size_t wall_x_extruder_nr,
                                         const ZSeamConfig& z_seam_config,
                                         const std::vector<VariableWidthLines>& paths)
    : gcode_writer(gcode_writer)
    , storage(storage)
    , gcode_layer(gcode_layer)
    , settings(settings)
    , extruder_nr(extruder_nr)
    , inset_0_non_bridge_config(inset_0_non_bridge_config)
    , inset_X_non_bridge_config(inset_X_non_bridge_config)
    , inset_0_bridge_config(inset_0_bridge_config)
    , inset_X_bridge_config(inset_X_bridge_config)
    , retract_before_outer_wall(retract_before_outer_wall)
    , wall_0_wipe_dist(wall_0_wipe_dist)
    , wall_x_wipe_dist(wall_x_wipe_dist)
    , wall_0_extruder_nr(wall_0_extruder_nr)
    , wall_x_extruder_nr(wall_x_extruder_nr)
    , z_seam_config(z_seam_config)
    , paths(paths)
    , layer_nr(gcode_layer.getLayerNr())
    , added_something(false)
    , retraction_region_calculated(false)
{
}

bool InsetOrderOptimizer::addToLayer()
{
    // Settings & configs:
    const auto pack_by_inset = ! settings.get<bool>("optimize_wall_printing_order");
    const auto inset_direction = settings.get<InsetDirection>("inset_direction");
    const auto alternate_walls = settings.get<bool>("material_alternate_walls");

    const bool outer_to_inner = inset_direction == InsetDirection::OUTSIDE_IN;
    const bool use_one_extruder = wall_0_extruder_nr == wall_x_extruder_nr;
    const bool current_extruder_is_wall_x = wall_x_extruder_nr == extruder_nr;

    const bool reverse = shouldReversePath(use_one_extruder, current_extruder_is_wall_x, outer_to_inner);
    auto walls_to_be_added = getWallsToBeAdded(reverse, use_one_extruder);

    const auto order = pack_by_inset ? getInsetOrder(walls_to_be_added, outer_to_inner) : getRegionOrder(walls_to_be_added, outer_to_inner);

    constexpr Ratio flow = 1.0_r;

    bool added_something = false;

    constexpr bool detect_loops = false;
    constexpr Polygons* combing_boundary = nullptr;
    // When we alternate walls, also alternate the direction at which the first wall starts in.
    // On even layers we start with normal direction, on odd layers with inverted direction.
    constexpr bool reverse_all_paths = false;
    PathOrderOptimizer<const ExtrusionLine*> order_optimizer(gcode_layer.getLastPlannedPositionOrStartingPosition(), z_seam_config, detect_loops, combing_boundary, reverse_all_paths, order);

    for (const auto& line : walls_to_be_added)
    {
        if (line.is_closed)
        {
            order_optimizer.addPolygon(&line);
        }
        else
        {
            order_optimizer.addPolyline(&line);
        }
    }


    order_optimizer.optimize();

    cura::Point p_end{ 0, 0 };
    for (const PathOrderPath<const ExtrusionLine*>& path : order_optimizer.paths)
    {
        if (path.vertices->empty())
            continue;

        const bool is_outer_wall = path.vertices->inset_idx == 0; // or thin wall 'gap filler'
        const bool is_gap_filler = path.vertices->is_odd;
        const GCodePathConfig& non_bridge_config = is_outer_wall ? inset_0_non_bridge_config : inset_X_non_bridge_config;
        const GCodePathConfig& bridge_config = is_outer_wall ? inset_0_bridge_config : inset_X_bridge_config;
        const coord_t wipe_dist = is_outer_wall && ! is_gap_filler ? wall_0_wipe_dist : wall_x_wipe_dist;
        const bool retract_before = is_outer_wall ? retract_before_outer_wall : false;

        const bool revert_inset = alternate_walls && (path.vertices->inset_idx % 2);
        const bool revert_layer = alternate_walls && (layer_nr % 2);
        const bool backwards = path.backwards != (revert_inset != revert_layer);
        const size_t start_index = (backwards != path.backwards) ? path.vertices->size() - (path.start_vertex + 1) : path.start_vertex;

        p_end = path.backwards ? path.vertices->back().p : path.vertices->front().p;
        const cura::Point p_start = path.backwards ? path.vertices->front().p : path.vertices->back().p;
        const bool linked_path = p_start != p_end;

        gcode_writer.setExtruder_addPrime(storage, gcode_layer, extruder_nr);
        gcode_layer.setIsInside(true); // Going to print walls, which are always inside.
        gcode_layer.addWall(*path.vertices, start_index, settings, non_bridge_config, bridge_config, wipe_dist, flow, retract_before, path.is_closed, backwards, linked_path);
        added_something = true;
    }
    return added_something;
}

std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> InsetOrderOptimizer::getRegionOrder(std::vector<ExtrusionLine>& input, const bool outer_to_inner)
{
    struct Loco
    {
        ExtrusionLine* line;
        Polygons poly;
        coord_t width;
        coord_t length;
        size_t idx;
    };

    struct Candidate
    {
        std::shared_ptr<Loco> candidate;
        double weight;
    };

    using map_t = std::unordered_multimap<std::shared_ptr<Loco>, Candidate>;
    map_t map{};

    constexpr auto offset = 40;  // TODO: base it on the max bead width?

    std::vector<std::shared_ptr<Loco>> locos;
    locos.reserve(input.size());
    for (auto [i, line] : input | ranges::views::enumerate)
    {
        Polygons poly;
        poly.emplace_back( line.toPolygon() );
        auto width = line.getMinimalWidth() / 2;
        auto prime_candidate = std::make_shared<Loco>(Loco{
            .line = &line,
            .poly = poly,
            .width = width,
            .length = line.getLength(),
            .idx = i,
        });
        locos.emplace_back(prime_candidate);
    }

    for (const auto& loco : locos)
    {
        for (const auto& candidate : locos | ranges::views::remove_if([loco](auto candidate){ return candidate == loco; }))
        {
            // Note: Current implementation favors outer intersections over inner intersections: `outer_radius > inner_radius`
            //  not sure yet if this works in our favor or has a negative effect. Need to doublecheck.
            AABB loco_box {loco->poly};
            AABB candidate_box {candidate->poly};
            if (! candidate_box.contains(loco_box))
            {
                auto area = loco->poly.offset(loco->width).intersection(candidate->poly.offset(-candidate->width)).area();
                double weight = area; // TODO: normalize the weight / loco->length; // Empty ExtrusionLines are already filtered out, no need to guard against 0-length
                if (weight > 0)
                {
                    map.emplace(std::make_pair(loco, Candidate{ .candidate = candidate, .weight = weight }));
                }
            }
        }
    }

    // Pick the best candidate
    std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> ret;
    for (const auto& loco : locos | ranges::views::unique)
    {
        auto [candidate_begin, candidate_end] = map.equal_range(loco);
        auto candidates = ranges::subrange(candidate_begin, candidate_end) | ranges::views::values | ranges::to_vector;
        if (! ranges::empty(candidates))
        {
            // TODO: pick the best candidate in the list also taking other loco's into account
            ranges::sort(candidates, ranges::greater{}, &Candidate::weight);
            auto best = ranges::max(candidates, ranges::less{}, &Candidate::weight);
            spdlog::debug("inset: {} -> {}", loco->idx, candidates | ranges::views::transform([](const auto& c){ return std::make_pair( c.candidate->idx, c.weight ); }));
            ret.emplace(loco->line, best.candidate->line);
            spdlog::debug("inset: {} -> {} with weight {}", loco->idx, best.candidate->idx, best.weight);
        }
    }
    return ret;
}

std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> InsetOrderOptimizer::getInsetOrder(const auto& input, const bool outer_to_inner)
{
    std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> order;

    std::vector<std::vector<const ExtrusionLine*>> walls_by_inset;
    std::vector<std::vector<const ExtrusionLine*>> fillers_by_inset;

    for (const auto& line : input)
    {
        if (line.is_odd)
        {
            if (line.inset_idx >= fillers_by_inset.size())
            {
                fillers_by_inset.resize(line.inset_idx + 1);
            }
            fillers_by_inset[line.inset_idx].emplace_back(&line);
        }
        else
        {
            if (line.inset_idx >= walls_by_inset.size())
            {
                walls_by_inset.resize(line.inset_idx + 1);
            }
            walls_by_inset[line.inset_idx].emplace_back(&line);
        }
    }
    for (size_t inset_idx = 0; inset_idx + 1 < walls_by_inset.size(); inset_idx++)
    {
        for (const ExtrusionLine* line : walls_by_inset[inset_idx])
        {
            for (const ExtrusionLine* inner_line : walls_by_inset[inset_idx + 1])
            {
                const ExtrusionLine* before = inner_line;
                const ExtrusionLine* after = line;
                if (outer_to_inner)
                {
                    std::swap(before, after);
                }
                order.emplace(before, after);
            }
        }
    }
    for (size_t inset_idx = 1; inset_idx < fillers_by_inset.size(); inset_idx++)
    {
        for (const ExtrusionLine* line : fillers_by_inset[inset_idx])
        {
            if (inset_idx - 1 >= walls_by_inset.size())
                continue;
            for (const ExtrusionLine* enclosing_wall : walls_by_inset[inset_idx - 1])
            {
                order.emplace(enclosing_wall, line);
            }
        }
    }

    return order;
}

constexpr bool InsetOrderOptimizer::shouldReversePath(const bool use_one_extruder, const bool current_extruder_is_wall_x, const bool outer_to_inner)
{
    if (use_one_extruder && current_extruder_is_wall_x)
    {
        return ! outer_to_inner;
    }
    return current_extruder_is_wall_x;
}

std::vector<ExtrusionLine> InsetOrderOptimizer::getWallsToBeAdded(const bool reverse, const bool use_one_extruder)
{
    ranges::any_view<VariableWidthLines> view;
    if (reverse)
    {
        if (use_one_extruder)
        {
            view = paths | ranges::views::reverse;
        }
        else
        {
            view = paths | ranges::views::reverse | ranges::views::drop_last(1);
        }
    }
    else
    {
        if (use_one_extruder)
        {
            view = paths | ranges::views::all;
        }
        else
        {
            view = paths | ranges::views::take_exactly(1);
        }
    }
    return view | ranges::views::join | ranges::views::remove_if(ranges::empty) | ranges::to_vector;
}
} // namespace cura
