//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ExtruderTrain.h"
#include "FffGcodeWriter.h"
#include "InsetOrderOptimizer.h"
#include "LayerPlan.h"
#include "utils/logoutput.h"
#include "WallToolPaths.h"

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
    const bool pack_by_inset = ! settings.get<bool>("optimize_wall_printing_order");
    const InsetDirection inset_direction = settings.get<InsetDirection>("inset_direction");
    const bool center_last = settings.get<bool>("wall_order_center_last");
    const bool alternate_walls = settings.get<bool>("material_alternate_walls");

    const bool outer_to_inner = inset_direction == InsetDirection::OUTSIDE_IN;
    
    size_t start_inset;
    size_t end_inset;
    int direction;
    //If the entire wall is printed with the current extruder, print all of it.
    if (wall_0_extruder_nr == wall_x_extruder_nr && wall_x_extruder_nr == extruder_nr)
    {
        //If printing the outer inset first, start with the lowest inset.
        //Otherwise start with the highest inset and iterate backwards.
        if(inset_direction == InsetDirection::OUTSIDE_IN)
        {
            start_inset = 0;
            end_inset = paths.size();
            direction = 1;
        }
        else //INSIDE_OUT or CENTER_LAST.
        {
            start_inset = paths.size() - 1;
            end_inset = -1;
            direction = -1;
        }
    }
    //If the wall is partially printed with the current extruder, print the correct part.
    else if (wall_0_extruder_nr != wall_x_extruder_nr)
    {
        //If the wall_0 and wall_x extruders are different, then only include the insets that should be printed by the
        //current extruder_nr.
        if(extruder_nr == wall_0_extruder_nr)
        {
            start_inset = 0;
            end_inset = 1; // Ignore inner walls
            direction = 1;
        }
        else if(extruder_nr == wall_x_extruder_nr)
        {
            start_inset = paths.size() - 1;
            end_inset = 0; // Ignore outer wall
            direction = -1;
        }
        else
        {
            return added_something;
        }
    }
    else //The wall is not printed with this extruder, not even in part. Don't print anything then.
    {
        return added_something;
    }

    
    std::vector<const ExtrusionLine*> walls_to_be_added;
    //Add all of the insets one by one.
    for (size_t inset_idx = start_inset; inset_idx != end_inset; inset_idx += direction)
    {
        if (paths[inset_idx].empty())
        {
            continue; //Don't switch extruders either, etc.
        }
        const VariableWidthLines& inset = paths[inset_idx];
        for (const ExtrusionLine& wall : inset)
        {
            walls_to_be_added.emplace_back(&wall);
        }
    }
    
    
    std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> order = 
        pack_by_inset?
        getInsetOrder(walls_to_be_added, outer_to_inner)
        : getRegionOrder(walls_to_be_added, outer_to_inner);
    
    if (center_last)
    {
        for (const ExtrusionLine* line : walls_to_be_added)
        {
            if (line->is_odd)
            {
                for (const ExtrusionLine* other_line : walls_to_be_added)
                {
                    if ( ! other_line->is_odd)
                    {
                        order.emplace(std::make_pair(other_line, line));
                    }
                }
            }
        }
    }
    
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
