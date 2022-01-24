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
    
    
    std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> order = getWeakOrder(walls_to_be_added, outer_to_inner);
    
    if (center_last)
    {
        for (const ExtrusionLine* line : walls_to_be_added)
            if (line->is_odd)
                for (const ExtrusionLine* other_line : walls_to_be_added)
                    if ( ! other_line->is_odd)
                        order.emplace(std::make_pair(other_line, line));
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




std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> InsetOrderOptimizer::getWeakOrder(const std::vector<const ExtrusionLine*>& input, const bool outer_to_inner)
{
    size_t max_inset_idx = 0;
    Polygons all_polygons;
    std::unordered_map<size_t, const ExtrusionLine*> poly_idx_to_extrusionline;
    for (const ExtrusionLine* line_p : input)
    {
        const ExtrusionLine& line = *line_p;
        if (line.empty()) continue;
        max_inset_idx = std::max(max_inset_idx, line.inset_idx);
        if ( ! shorterThan(line.front().p - line.back().p, coincident_point_distance)) // TODO: check if it is a closed polygon or not
        {
            // Make a small triangle representative of the polyline
            // otherwise the polyline would get erased by the clipping operation
            all_polygons.emplace_back();
            assert(line.junctions.size() >= 2);
            Point middle = ( line.junctions[line.junctions.size() / 2 - 1].p + line.junctions[line.junctions.size() / 2].p ) / 2;
            PolygonRef poly = all_polygons.back();
            poly.emplace_back(middle);
            poly.emplace_back(middle + Point(5, 0));
            poly.emplace_back(middle + Point(0, 5));
        }
        else
        {
            all_polygons.emplace_back(line.toPolygon());
        }
        poly_idx_to_extrusionline.emplace(all_polygons.size() - 1, &line);
    }

    std::vector<std::vector<size_t>> nesting = all_polygons.getNesting();

    std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> result;

    for (auto [idx, line] : poly_idx_to_extrusionline)
    { // there might be multiple roots
        if (line->inset_idx == 0)
        {
            getWeakOrder(idx, poly_idx_to_extrusionline, nesting, max_inset_idx, outer_to_inner, result);
        }
    }

    return result;
}

void InsetOrderOptimizer::getWeakOrder(size_t node_idx, const std::unordered_map<size_t, const ExtrusionLine*>& poly_idx_to_extrusionline, const std::vector<std::vector<size_t>>& nesting, size_t max_inset_idx, const bool outer_to_inner, std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>>& result)
{
    auto parent_it = poly_idx_to_extrusionline.find(node_idx);
    assert(parent_it != poly_idx_to_extrusionline.end());
    const ExtrusionLine* parent = parent_it->second;
    
    assert(node_idx < nesting.size());
    for (size_t child_idx : nesting[node_idx])
    {
        auto child_it = poly_idx_to_extrusionline.find(child_idx);
        assert(child_it != poly_idx_to_extrusionline.end());
        const ExtrusionLine* child = child_it->second;

        if ( ! child->is_odd && child->inset_idx == parent->inset_idx && child->inset_idx == max_inset_idx)
        {
            // There is no order requirement between the innermost wall of a hole and the innermost wall of the outline.
        }
        else if ( ! child->is_odd && child->inset_idx == parent->inset_idx && child->inset_idx <= max_inset_idx)
        { // unusual case
            // There are insets with one higher inset index which are adjacent to both this child and the parent.
            // And potentially also insets which are adjacent to this child and other children.
            // Moreover there are probably gap filler lines in between the child and the parent.
            // The nesting information doesn't tell which ones are adjacent,
            // so just to be safe we add order requirements between the child and all gap fillers and wall lines.
            for (size_t other_child_idx : nesting[node_idx])
            {
                auto other_child_it = poly_idx_to_extrusionline.find(other_child_idx);
                assert(other_child_it != poly_idx_to_extrusionline.end());
                const ExtrusionLine* other_child = other_child_it->second;

                if (other_child == child) continue;
                
                
                // See if there's an overlap in region_id.
                // If not then they are not adjacent, so we don't include order requirement
                bool overlap = false;
                {
                    std::unordered_set<size_t> other_child_region_ids;
                    for (const ExtrusionJunction& j : other_child->junctions)
                    {
                        other_child_region_ids.emplace(j.region_id);
                    }
                    for (const ExtrusionJunction& j : child->junctions)
                    {
                        if (other_child_region_ids.count(j.region_id))
                        {
                            overlap = true;
                            break;
                        }
                    }
                    if (other_child->is_odd)
                    { // Odd gap fillers should have two region_ids, but they don't, so let's be more conservative on them
                        for (const ExtrusionJunction& j : parent->junctions)
                        {
                            if (other_child_region_ids.count(j.region_id))
                            { // if an odd gap filler has the region_id set to the outline then it could also be adjacent to child, but not registered as such.
                                overlap = true;
                                break;
                            }
                        }
                    }
                }
                if ( ! overlap) continue;
                if (other_child->is_odd)
                {
                    if (other_child->inset_idx == child->inset_idx + 1)
                    { // normal gap filler
                        result.emplace(child, other_child);
                    }
                    else
                    { // outer thin wall 'gap filler' enclosed in an internal hole.
                        // E.g. in the middle of a thin '8' shape when the middle looks like '>-<=>-<'
                        assert(parent->inset_idx == 0); // enclosing 8 shape
                        assert(child->inset_idx == 0); // thick section of the middle
                        assert(other_child->inset_idx == 0); // thin section of the middle
                        // no order requirement between thin wall, because it has no eclosing wall
                    }
                }
                else
                { // other child is an even wall as well
                    if (other_child->inset_idx == child->inset_idx) continue;
                    assert(other_child->inset_idx == child->inset_idx + 1);

                    const ExtrusionLine* before = child;
                    const ExtrusionLine* after = other_child;
                    if ( ! outer_to_inner)
                    {
                        std::swap(before, after);
                    }
                    result.emplace(before, after);
                }
            }
        }
        else
        { // normal case
            assert( ! parent->is_odd && "There can be no polygons inside a polyline");

            const ExtrusionLine* before = parent;
            const ExtrusionLine* after = child;
            if ( (child->inset_idx < parent->inset_idx) == outer_to_inner
                // ^ Order should be reversed for hole polyons
                // and it should be reversed again when the global order is the other way around
                && ! child->is_odd) // Odd polylines should always go after their enclosing wall polygon
            {
                std::swap(before, after);
            }
            result.emplace(before, after);
        }

        // Recurvise call
        getWeakOrder(child_idx, poly_idx_to_extrusionline, nesting, max_inset_idx, outer_to_inner, result);
    }
}



}//namespace cura
