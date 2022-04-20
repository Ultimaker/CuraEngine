//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ExtruderTrain.h"
#include "FffGcodeWriter.h"
#include "InsetOrderOptimizer.h"
#include "LayerPlan.h"
#include "utils/logoutput.h"
#include "WallToolPaths.h"

#include <iterator>

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
                                         const std::vector<VariableWidthLines>& paths) :
    gcode_writer(gcode_writer),
    storage(storage),
    gcode_layer(gcode_layer),
    settings(settings),
    extruder_nr(extruder_nr),
    inset_0_non_bridge_config(inset_0_non_bridge_config),
    inset_X_non_bridge_config(inset_X_non_bridge_config),
    inset_0_bridge_config(inset_0_bridge_config),
    inset_X_bridge_config(inset_X_bridge_config),
    retract_before_outer_wall(retract_before_outer_wall),
    wall_0_wipe_dist(wall_0_wipe_dist),
    wall_x_wipe_dist(wall_x_wipe_dist),
    wall_0_extruder_nr(wall_0_extruder_nr),
    wall_x_extruder_nr(wall_x_extruder_nr),
    z_seam_config(z_seam_config),
    paths(paths),
    layer_nr(gcode_layer.getLayerNr()),
    added_something(false),
    retraction_region_calculated(false)
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

    auto should_reverse = [&](){
        if (use_one_extruder && current_extruder_is_wall_x)
        {
            // The entire wall is printed with the current extruder.
            // Reversing the insets now depends on the inverse of the inset direction.
            // If we want to print the outer insets first we start with the lowest and move forward
            // otherwise we start with the highest and iterate back.
            return ! outer_to_inner;
        }
        // If the wall is partially printed with the current extruder we need to move forward
        // for the outer wall extruder and iterate back for the inner wall extruder
        return ! current_extruder_is_wall_x;
    };  // Helper lambda to ensure that the reverse bool can be a const type
    const bool reverse = should_reverse();

    // Switches the begin()...end() forward iterator for a rbegin()...rend() reverse iterator
    // I can't wait till we use the C++20 standard and have access to ranges and views
    auto get_walls_to_be_added = [&](const bool reverse, const std::vector<VariableWidthLines>& paths)
    {
        if (paths.empty())
        {
            return std::vector<const ExtrusionLine*>{};
        }
        if (reverse)
        {
            if (use_one_extruder)
            {
                return wallsToBeAdded(paths.rbegin(), paths.rend()); // Complete wall with one extruder
            }
            return wallsToBeAdded(std::next(paths.rbegin()), paths.rend()); // Ignore outer wall
        }
        if (use_one_extruder)
        {
            return wallsToBeAdded(paths.begin(), paths.end()); // Complete wall with one extruder
        }
        return wallsToBeAdded(std::next(paths.begin()), paths.end()); // Ignore inner wall
    };
    auto walls_to_be_added = get_walls_to_be_added(reverse, paths);

    auto order = pack_by_inset ? getInsetOrder(walls_to_be_added, outer_to_inner) : getRegionOrder(walls_to_be_added, outer_to_inner);
    
    constexpr Ratio flow = 1.0_r;
    
    bool added_something = false;

    constexpr bool detect_loops = false;
    constexpr Polygons* combing_boundary = nullptr;
    //When we alternate walls, also alternate the direction at which the first wall starts in.
    //On even layers we start with normal direction, on odd layers with inverted direction.
    constexpr bool reverse_all_paths = false;
    PathOrderOptimizer<const ExtrusionLine*> order_optimizer(gcode_layer.getLastPlannedPositionOrStartingPosition(), z_seam_config, detect_loops, combing_boundary, reverse_all_paths, order);
    
    for (const ExtrusionLine* line : walls_to_be_added)
    {
        if (line->is_closed)
        {
            order_optimizer.addPolygon(line);
        }
        else
        {
            order_optimizer.addPolyline(line);
        }
    }
    
    
    order_optimizer.optimize();
    
    cura::Point p_end {0, 0};
    for(const PathOrderPath<const ExtrusionLine*>& path : order_optimizer.paths)
    {
        if (path.vertices->empty()) continue;
        
        const bool is_outer_wall = path.vertices->inset_idx == 0; // or thin wall 'gap filler'
        const bool is_gap_filler = path.vertices->is_odd;
        const GCodePathConfig& non_bridge_config = is_outer_wall ? inset_0_non_bridge_config : inset_X_non_bridge_config;
        const GCodePathConfig& bridge_config = is_outer_wall? inset_0_bridge_config : inset_X_bridge_config;
        const coord_t wipe_dist = is_outer_wall && ! is_gap_filler ? wall_0_wipe_dist : wall_x_wipe_dist;
        const bool retract_before = is_outer_wall ? retract_before_outer_wall : false;

        const bool revert_inset = alternate_walls && (path.vertices->inset_idx % 2);
        const bool revert_layer = alternate_walls && (layer_nr % 2);
        const bool backwards = path.backwards != (revert_inset != revert_layer);
        
        p_end = path.backwards ? path.vertices->back().p : path.vertices->front().p;
        const cura::Point p_start = path.backwards ? path.vertices->front().p : path.vertices->back().p;
        const bool linked_path = p_start != p_end;

        gcode_writer.setExtruder_addPrime(storage, gcode_layer, extruder_nr);
        gcode_layer.setIsInside(true); //Going to print walls, which are always inside.
        gcode_layer.addWall(*path.vertices, path.start_vertex, settings, non_bridge_config, bridge_config, wipe_dist, flow, retract_before, path.is_closed, backwards, linked_path);
        added_something = true;
    }
    return added_something;
}




std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> InsetOrderOptimizer::getRegionOrder(const std::vector<const ExtrusionLine*>& input, const bool outer_to_inner)
{
    std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> order_requirements;

    // We build a grid where we map toolpath vertex locations to toolpaths,
    // so that we can easily find which two toolpaths are next to each other,
    // which is the requirement for there to be an order constraint.
    // 
    // We use a PointGrid rather than a LineGrid to save on computation time.
    // In very rare cases two insets might lie next to each other without having neighboring vertices, e.g.
    //  \            .
    //   |  /        .
    //   | /         .
    //   ||          .
    //   | \         .
    //   |  \        .
    //  /            .
    // However, because of how Arachne works this will likely never be the case for two consecutive insets.
    // On the other hand one could imagine that two consecutive insets of a very large circle
    // could be simplify()ed such that the remaining vertices of the two insets don't align.
    // In those cases the order requirement is not captured,
    // which means that the PathOrderOptimizer *might* result in a violation of the user set path order.
    // This problem is expected to be not so severe and happen very sparsely.

    coord_t max_line_w = 0u;
    for (const ExtrusionLine* line : input)
    { // compute max_line_w
        for (const ExtrusionJunction& junction : *line)
        {
            max_line_w = std::max(max_line_w, junction.w);
        }
    }
    if (max_line_w == 0u) return order_requirements;
    
    struct LineLoc
    {
        ExtrusionJunction j;
        const ExtrusionLine* line;
    };
    struct Locator
    {
        Point operator()(const LineLoc& elem)
        {
            return elem.j.p;
        }
    };
    
    // How much farther two verts may be apart due to corners.
    // This distance must be smaller than 2, because otherwise
    // we could create an order requirement between e.g.
    // wall 2 of one region and wall 3 of another region,
    // while another wall 3 of the first region would lie in between those two walls.
    // However, higher values are better against the limitations of using a PointGrid rather than a LineGrid.
    constexpr float diagonal_extension = 1.9;
    const coord_t searching_radius = max_line_w * diagonal_extension;
    using GridT = SparsePointGrid<LineLoc, Locator>;
    GridT grid(searching_radius);

    
    for (const ExtrusionLine* line : input)
    {
        for (const ExtrusionJunction& junction : *line)
        {
            grid.insert(LineLoc{junction, line});
        }
    }
    for (const std::pair<SquareGrid::GridPoint, LineLoc>& pair : grid)
    {
        const LineLoc& lineloc_here = pair.second;
        const ExtrusionLine* here = lineloc_here.line;
        Point loc_here = pair.second.j.p;
        std::vector<LineLoc> nearby_verts = grid.getNearby(loc_here, searching_radius);
        for (const LineLoc& lineloc_nearby : nearby_verts)
        {
            const ExtrusionLine* nearby = lineloc_nearby.line;
            if (nearby == here) continue;
            if (nearby->inset_idx == here->inset_idx) continue;
            if (nearby->inset_idx > here->inset_idx + 1) continue; // not directly adjacent
            if (here->inset_idx > nearby->inset_idx + 1) continue; // not directly adjacent
            if ( ! shorterThan(loc_here - lineloc_nearby.j.p, (lineloc_here.j.w + lineloc_nearby.j.w) / 2 * diagonal_extension)) continue; // points are too far away from each other
            if (here->is_odd || nearby->is_odd)
            {
                if (here->is_odd && ! nearby->is_odd && nearby->inset_idx < here->inset_idx)
                {
                    order_requirements.emplace(std::make_pair(nearby, here));
                }
                if (nearby->is_odd && ! here->is_odd && here->inset_idx < nearby->inset_idx)
                {
                    order_requirements.emplace(std::make_pair(here, nearby));
                }
            }
            else if ((nearby->inset_idx < here->inset_idx) == outer_to_inner)
            {
                order_requirements.emplace(std::make_pair(nearby, here));
            }
            else
            {
                assert((nearby->inset_idx > here->inset_idx) == outer_to_inner);
                order_requirements.emplace(std::make_pair(here, nearby));
            }
        }
    }
    return order_requirements;
}

std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> InsetOrderOptimizer::getInsetOrder(const std::vector<const ExtrusionLine*>& input, const bool outer_to_inner)
{
    std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> order;
    
    std::vector<std::vector<const ExtrusionLine*>> walls_by_inset;
    std::vector<std::vector<const ExtrusionLine*>> fillers_by_inset;

    for (const ExtrusionLine* line : input)
    {
        if (line->is_odd)
        {
            if (line->inset_idx >= fillers_by_inset.size())
            {
                fillers_by_inset.resize(line->inset_idx + 1);
            }
            fillers_by_inset[line->inset_idx].emplace_back(line);
        }
        else
        {
            if (line->inset_idx >= walls_by_inset.size())
            {
                walls_by_inset.resize(line->inset_idx + 1);
            }
            walls_by_inset[line->inset_idx].emplace_back(line);
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
            if (inset_idx - 1 >= walls_by_inset.size()) continue;
            for (const ExtrusionLine* enclosing_wall : walls_by_inset[inset_idx - 1])
            {
                order.emplace(enclosing_wall, line);
            }
        }
        
    }
    
    return order;
}


}//namespace cura
