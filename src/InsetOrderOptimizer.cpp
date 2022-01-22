//Copyright (c) 2021 Ultimaker B.V.
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
                                         const VariableWidthPaths& paths) :
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
    const bool pack_by_inset = ! settings.get<bool>("optimize_wall_printing_order"); // TODO
    const InsetDirection inset_direction = settings.get<InsetDirection>("inset_direction");
    const bool center_last = inset_direction == InsetDirection::CENTER_LAST;
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
    for(size_t inset_idx = start_inset; inset_idx != end_inset; inset_idx += direction)
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
    
    
    constexpr bool include_transitive = true;
    std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> order = WallToolPaths::getWeakOrder(walls_to_be_added, outer_to_inner, include_transitive);
    
    if (center_last)
    {
        for (const ExtrusionLine* line : walls_to_be_added)
            if (line->is_odd)
                for (const ExtrusionLine* other_line : walls_to_be_added)
                    if ( ! other_line->is_odd)
                        order.emplace(std::make_pair(other_line, line));
    }
    
    using Optimizer = PathOrderOptimizer<const ExtrusionLine*>;
    std::function<bool (const Optimizer::Path&, const Optimizer::Path&)> canPrecede =
        [&order](const Optimizer::Path& before, const Optimizer::Path& after)
        {
            // [before] cannot precede [after] if we have an order constraint that [after] must be before [before]
            return ! order.count(std::make_pair(after.vertices, before.vertices));
        };
#ifdef DEBUG
    {
        AABB aabb;
        for (auto& inset : paths)
            for (auto& line : inset)
                for (auto p : line)
                    aabb.include(p.p);
        SVG svg("/tmp/order.svg", aabb);
        for (auto& inset : paths)
            for (auto& line : inset)
                svg.writePolyline(line.toPolygon(), line.is_odd? SVG::Color::RED : SVG::Color::GREEN);
        svg.nextLayer();
        for (auto& inset : paths)
            for (auto& line : inset)
                svg.writePoints(line.toPolygon(), true, 1.0);
        svg.nextLayer();
        for (auto [before, after] : order)
            if ( ! after->is_odd)
                svg.writeArrow(before->junctions[1 % before->junctions.size()].p, after->junctions[2 % after->junctions.size()].p, SVG::Color::BLUE);
        svg.nextLayer();
        for (auto [before, after] : order)
            if (after->is_odd)
                svg.writeArrow(before->junctions[1 % before->junctions.size()].p, after->junctions[2 % after->junctions.size()].p, SVG::Color::MAGENTA);
    }
#endif // DEBUG
    
    constexpr Ratio flow = 1.0_r;
    
    bool added_something = false;

    constexpr bool detect_loops = true;
    constexpr Polygons* combing_boundary = nullptr;
    //When we alternate walls, also alternate the direction at which the first wall starts in.
    //On even layers we start with normal direction, on odd layers with inverted direction.
    constexpr bool reverse_all_paths = false;
    constexpr bool selection_optimization = false;
    PathOrderOptimizer<const ExtrusionLine*> order_optimizer(gcode_layer.getLastPlannedPositionOrStartingPosition(), z_seam_config, detect_loops, combing_boundary, reverse_all_paths, selection_optimization, canPrecede);
    
    for (const ExtrusionLine* line : walls_to_be_added)
    {
        order_optimizer.addPolyline(line);
    }
    
    
    order_optimizer.optimize();
    
    cura::Point p_end {0, 0};
    for(const PathOrderOptimizer<const ExtrusionLine*>::Path& path : order_optimizer.paths)
    {
        if (path.vertices->empty()) continue;
        
        const bool is_outer_wall = path.vertices->inset_idx == 0; // or thin wall 'gap filler'
        const bool is_gap_filler = path.vertices->is_odd;
        const GCodePathConfig& non_bridge_config = is_outer_wall ? inset_0_non_bridge_config : inset_X_non_bridge_config;
        const GCodePathConfig& bridge_config = is_outer_wall? inset_0_bridge_config : inset_X_bridge_config;
        const coord_t wipe_dist = is_outer_wall && ! is_gap_filler ? wall_0_wipe_dist : wall_x_wipe_dist;
        const bool retract_before = is_outer_wall ? retract_before_outer_wall : false;
        
        const bool alternate_direction_modifier = alternate_walls && (path.vertices->inset_idx % 2 == layer_nr % 2);
        const bool backwards = path.backwards != alternate_direction_modifier;
        
        p_end = path.backwards ? path.vertices->back().p : path.vertices->front().p;
        const cura::Point p_start = path.backwards ? path.vertices->front().p : path.vertices->back().p;
        const bool linked_path = p_start != p_end;
        
        
        added_something = true;
        gcode_writer.setExtruder_addPrime(storage, gcode_layer, extruder_nr);
        gcode_layer.setIsInside(true); //Going to print walls, which are always inside.
        gcode_layer.addWall(*path.vertices, path.start_vertex, settings, non_bridge_config, bridge_config, wipe_dist, flow, retract_before, path.is_closed, backwards, linked_path);
        added_something = true;
    }
    return added_something;
}

}//namespace cura
