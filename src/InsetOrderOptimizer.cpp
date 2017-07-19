//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "InsetOrderOptimizer.h"

namespace cura
{

static int findAdjacentEnclosingPoly(const ConstPolygonRef& enclosed_inset, const std::vector<ConstPolygonRef>& possible_enclosing_polys, const coord_t max_gap)
{
    // given an inset, search a collection of insets for the adjacent enclosing inset
    Polygons enclosed;
    enclosed.add(enclosed_inset);
    for (unsigned enclosing_poly_idx = 0; enclosing_poly_idx < possible_enclosing_polys.size(); ++enclosing_poly_idx)
    {
        Polygons enclosing;
        enclosing.add(possible_enclosing_polys[enclosing_poly_idx]);
        // as holes don't overlap, if the insets intersect, it is safe to assume that the enclosed inset is inside the enclosing inset
        if (PolygonUtils::polygonsIntersect(enclosing, enclosed) && PolygonUtils::polygonOutlinesAdjacent(enclosed_inset, enclosing[0], max_gap))
        {
            return enclosing_poly_idx;
        }
    }
    return -1;
}

void InsetOrderOptimizer::processHoleInsets()
{
    const coord_t wall_line_width_0 = mesh_config.inset0_config.getLineWidth();
    const coord_t wall_line_width_x = mesh_config.insetX_config.getLineWidth();
    const coord_t wall_0_wipe_dist = mesh.getSettingInMicrons("wall_0_wipe_dist");
    const bool compensate_overlap_0 = mesh.getSettingBoolean("travel_compensate_overlapping_walls_0_enabled");
    const bool compensate_overlap_x = mesh.getSettingBoolean("travel_compensate_overlapping_walls_x_enabled");
    const bool retract_before_outer_wall = mesh.getSettingBoolean("travel_retract_before_outer_wall");
    const bool outer_inset_first = mesh.getSettingBoolean("outer_inset_first")
        || (layer_nr == 0 && mesh.getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::BRIM && !mesh.getSettingBoolean("brim_outside_only"));
    const unsigned num_insets = part.insets.size();
    constexpr bool spiralize = false;
    constexpr float flow = 1.0;

    // work out the order we wish to visit all the holes (doesn't include the outer wall of the part)
    PathOrderOptimizer orderOptimizer(gcode_layer.getLastPosition(), z_seam_pos, z_seam_type);
    for (unsigned int poly_idx = 1; poly_idx < inset_polys[0].size(); poly_idx++)
    {
        orderOptimizer.addPolygon(inset_polys[0][poly_idx]);
    }
    orderOptimizer.optimize();

    // this will consume all of the insets that surround holes but not the insets next to the outermost wall of the model
    for (unsigned outer_poly_order_idx = 0; outer_poly_order_idx < orderOptimizer.polyOrder.size(); ++outer_poly_order_idx)
    {
        Polygons hole_outer_wall; // the outermost wall of a hole
        hole_outer_wall.add(inset_polys[0][orderOptimizer.polyOrder[outer_poly_order_idx] + 1]); // +1 because first element (part outer wall) wasn't included
        std::vector<unsigned> hole_level_1_wall_indices; // the indices of the walls that touch the hole's outer wall
        const coord_t max_gap = std::max(wall_line_width_0, wall_line_width_x) * 1.1f; // if polys are closer than this, they are considered adjacent
        if (inset_polys.size() > 1)
        {
            // find the adjacent poly in the level 1 insets that encloses the hole
            int adjacent_enclosing_poly_idx = findAdjacentEnclosingPoly(hole_outer_wall[0], inset_polys[1], max_gap);
            if (adjacent_enclosing_poly_idx >= 0)
            {
                // now test for the case where we are printing the outer walls first and this level 1 inset also touches other outer walls
                // in this situation we don't want to output the level 1 inset until after all the outer walls (that it touches) have been printed
                if (outer_inset_first)
                {
                    // does the level 1 inset touch more than one outline?
                    Polygons inset;
                    inset.add(inset_polys[1][adjacent_enclosing_poly_idx]);
                    int num_future_outlines_touched = 0; // number of outlines that have yet to be output that are touched by this level 1 inset
                    // does it touch the outer wall?
                    if (PolygonUtils::polygonOutlinesAdjacent(inset[0], inset_polys[0][0], max_gap))
                    {
                        // yes, the level 1 inset touches the part's outer wall
                        ++num_future_outlines_touched;
                    }
                    // does it touch any yet to be processed hole outlines?
                    for (unsigned order_index = outer_poly_order_idx + 1; num_future_outlines_touched < 1 && order_index < orderOptimizer.polyOrder.size(); ++order_index)
                    {
                        int outline_index = orderOptimizer.polyOrder[order_index] + 1; // +1 because first element (part outer wall) wasn't included
                        // as we don't know the shape of the outlines (straight, concave, convex, etc.) and the
                        // adjacency test assumes that the poly's are arranged so that the first has smaller
                        // radius curves than the second (it's "inside" the second) we need to test both combinations
                        if (PolygonUtils::polygonOutlinesAdjacent(inset[0], inset_polys[0][outline_index], max_gap) ||
                            PolygonUtils::polygonOutlinesAdjacent(inset_polys[0][outline_index], inset[0], max_gap))
                        {
                            // yes, it touches this yet to be processed hole outline
                            ++num_future_outlines_touched;
                        }
                    }
                    if (num_future_outlines_touched < 1)
                    {
                        // this level 1 inset only touches outlines that have already been processed so we can print it
                        hole_level_1_wall_indices.push_back(adjacent_enclosing_poly_idx);
                    }
                }
                else
                {
                    // print this level 1 inset
                    hole_level_1_wall_indices.push_back(adjacent_enclosing_poly_idx);
                }
            }
            else if (!outer_inset_first)
            {
                // we didn't find a level 1 inset that encloses this hole so now look to see if there is one or more level 1 insets that simply touch
                // this hole and use those instead - however, as the level 1 insets will also touch other holes and/or the outer wall we don't want
                // to do this when printing the outer walls first
                PolygonUtils::findAdjacentPolygons(hole_level_1_wall_indices, hole_outer_wall[0], inset_polys[1], max_gap);
            }
        }

        // now collect the inner walls that will be printed along with the hole outer wall

        Polygons hole_inner_walls; // the hole's inner walls

        for (unsigned level_1_wall_idx = 0; level_1_wall_idx < hole_level_1_wall_indices.size(); ++level_1_wall_idx)
        {
            // add the level 1 inset to the collection of inner walls to be printed and consume it, taking care to adjust
            // those elements in hole_level_1_wall_indices that are larger
            unsigned inset_idx = hole_level_1_wall_indices[level_1_wall_idx];
            ConstPolygonRef lastInset = inset_polys[1][inset_idx];
            hole_inner_walls.add(lastInset);
            inset_polys[1].erase(inset_polys[1].begin() + inset_idx);
            // decrement any other indices in hole_level_1_wall_indices that are greater than inset_idx
            for (unsigned i = level_1_wall_idx + 1; i < hole_level_1_wall_indices.size(); ++i)
            {
                if (hole_level_1_wall_indices[i] > inset_idx)
                {
                    hole_level_1_wall_indices[i] = hole_level_1_wall_indices[i] - 1;
                }
            }
            // now find all the insets that immediately surround the level 1 wall and consume them
            for (unsigned inset_level = 2; inset_level < num_insets && inset_polys[inset_level].size(); ++inset_level)
            {
                int i = findAdjacentEnclosingPoly(lastInset, inset_polys[inset_level], wall_line_width_x * 1.1f);
                if (i >= 0)
                {
                    // we have found an enclosing inset
                    if (outer_inset_first)
                    {
                        // when printing outer insets first we don't want to print this enclosing inset
                        // if it also encloses other holes that haven't yet been processed so check the holes
                        // that haven't yet been processed to see if they are also enclosed by this enclosing inset
                        Polygons enclosing_inset;
                        enclosing_inset.add(inset_polys[inset_level][i]);
                        bool encloses_future_hole = false; // set true if this inset also encloses another hole that hasn't yet been processed
                        for (unsigned hole_order_index = outer_poly_order_idx + 1; !encloses_future_hole && hole_order_index < orderOptimizer.polyOrder.size(); ++hole_order_index)
                        {
                            Polygons enclosed_inset;
                            enclosed_inset.add(inset_polys[0][orderOptimizer.polyOrder[hole_order_index] + 1]); // +1 because first element (part outer wall) wasn't included
                            encloses_future_hole = PolygonUtils::polygonsIntersect(enclosing_inset, enclosed_inset);
                        }
                        if (encloses_future_hole)
                        {
                            // give up finding the insets that surround this hole
                            break;
                        }
                    }
                    // it's OK to print the enclosing inset and see if any further enclosing insets can also be printed
                    lastInset = inset_polys[inset_level][i];
                    hole_inner_walls.add(lastInset);
                    inset_polys[inset_level].erase(inset_polys[inset_level].begin() + i);
                }
            }
        }

        if (hole_inner_walls.size() > 0 && extruder_nr == mesh.getSettingAsExtruderNr("wall_x_extruder_nr"))
        {
            // output the inset polys

            gcode_writer.setExtruder_addPrime(storage, gcode_layer, layer_nr, extruder_nr);
            gcode_layer.setIsInside(true); // going to print stuff inside print object
            if (outer_inset_first)
            {
                if (extruder_nr == mesh.getSettingAsExtruderNr("wall_0_extruder_nr"))
                {
                    if (compensate_overlap_0)
                    {
                        WallOverlapComputation wall_overlap_computation(hole_outer_wall, wall_line_width_0);
                        gcode_layer.addPolygonsByOptimizer(hole_outer_wall, mesh_config.inset0_config, &wall_overlap_computation, z_seam_type, z_seam_pos, wall_0_wipe_dist, spiralize, flow, retract_before_outer_wall);
                    }
                    else
                    {
                        WallOverlapComputation* wall_overlap_computation(nullptr);
                        gcode_layer.addPolygonsByOptimizer(hole_outer_wall, mesh_config.inset0_config, wall_overlap_computation, z_seam_type, z_seam_pos, wall_0_wipe_dist, spiralize, flow, retract_before_outer_wall);
                    }
                }
                if (compensate_overlap_x)
                {
                    WallOverlapComputation wall_overlap_computation(hole_inner_walls, wall_line_width_x);
                    gcode_layer.addPolygonsByOptimizer(hole_inner_walls, mesh_config.insetX_config, &wall_overlap_computation);
                }
                else
                {
                    gcode_layer.addPolygonsByOptimizer(hole_inner_walls, mesh_config.insetX_config);
                }
            }
            else
            {
                // when the user has specified the z seam location, we want the insets that surround the hole to start
                // as close as possible to the z seam location so to avoid the possible retract when moving from the end
                // of the immediately enclosing inset to the start of the hole outer wall we first move to a location
                // that is close to the z seam and at a vertex of the first inset we want to be printed
                if (z_seam_type == EZSeamType::USER_SPECIFIED)
                {
                    const Point z_seam_location = hole_outer_wall[0][orderOptimizer.polyStart[orderOptimizer.polyOrder[outer_poly_order_idx]]];
                    // move to the location of the vertex in the outermost enclosing inset that's closest to the z seam location
                    const Point dest = hole_inner_walls.back()[PolygonUtils::findNearestVert(z_seam_location, hole_inner_walls.back())];
                    gcode_layer.addTravel(dest);
                }
                std::reverse(hole_inner_walls.begin(), hole_inner_walls.end());
                if (compensate_overlap_x)
                {
                    WallOverlapComputation wall_overlap_computation(hole_inner_walls, wall_line_width_x);
                    gcode_layer.addPolygonsByOptimizer(hole_inner_walls, mesh_config.insetX_config, &wall_overlap_computation);
                }
                else
                {
                    gcode_layer.addPolygonsByOptimizer(hole_inner_walls, mesh_config.insetX_config);
                }
                if (extruder_nr == mesh.getSettingAsExtruderNr("wall_0_extruder_nr"))
                {
                    if (compensate_overlap_0)
                    {
                        WallOverlapComputation wall_overlap_computation(hole_outer_wall, wall_line_width_0);
                        gcode_layer.addPolygonsByOptimizer(hole_outer_wall, mesh_config.inset0_config, &wall_overlap_computation, z_seam_type, z_seam_pos, wall_0_wipe_dist, spiralize, flow, retract_before_outer_wall);
                    }
                    else
                    {
                        WallOverlapComputation* wall_overlap_computation(nullptr);
                        gcode_layer.addPolygonsByOptimizer(hole_outer_wall, mesh_config.inset0_config, wall_overlap_computation, z_seam_type, z_seam_pos, wall_0_wipe_dist, spiralize, flow, retract_before_outer_wall);
                    }
                }
            }
            added_something = true;
        }
        else if (extruder_nr == mesh.getSettingAsExtruderNr("wall_0_extruder_nr"))
        {
            // just the outer wall, no level 1 insets
            gcode_writer.setExtruder_addPrime(storage, gcode_layer, layer_nr, extruder_nr);
            gcode_layer.setIsInside(true); // going to print stuff inside print object
            if (compensate_overlap_0)
            {
                WallOverlapComputation wall_overlap_computation(hole_outer_wall, wall_line_width_0);
                gcode_layer.addPolygonsByOptimizer(hole_outer_wall, mesh_config.inset0_config, &wall_overlap_computation, z_seam_type, z_seam_pos, wall_0_wipe_dist, spiralize, flow, retract_before_outer_wall);
            }
            else
            {
                WallOverlapComputation* wall_overlap_computation(nullptr);
                gcode_layer.addPolygonsByOptimizer(hole_outer_wall, mesh_config.inset0_config, wall_overlap_computation, z_seam_type, z_seam_pos, wall_0_wipe_dist, spiralize, flow, retract_before_outer_wall);
            }
            added_something = true;
        }
    }
}

void InsetOrderOptimizer::processOuterWallInsets()
{
    const coord_t wall_line_width_0 = mesh_config.inset0_config.getLineWidth();
    const coord_t wall_line_width_x = mesh_config.insetX_config.getLineWidth();
    const coord_t wall_0_wipe_dist = mesh.getSettingInMicrons("wall_0_wipe_dist");
    const bool compensate_overlap_0 = mesh.getSettingBoolean("travel_compensate_overlapping_walls_0_enabled");
    const bool compensate_overlap_x = mesh.getSettingBoolean("travel_compensate_overlapping_walls_x_enabled");
    const bool retract_before_outer_wall = mesh.getSettingBoolean("travel_retract_before_outer_wall");
    const bool outer_inset_first = mesh.getSettingBoolean("outer_inset_first")
                                    || (layer_nr == 0 && mesh.getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::BRIM);
    const unsigned num_insets = part.insets.size();
    constexpr bool spiralize = false;
    constexpr float flow = 1.0;

    // process the part's outer wall and the level 1 insets that it surrounds
    {
        Polygons part_outer_wall; // the outermost wall of a part
        part_outer_wall.add(inset_polys[0][0]);
        // find the level 1 insets that are inside the outer wall and consume them
        Polygons part_inner_walls;
        int num_level_1_insets = 0;
        for (unsigned level_1_wall_idx = 0; inset_polys.size() > 1 && level_1_wall_idx < inset_polys[1].size(); ++level_1_wall_idx)
        {
            Polygons inner;
            inner.add(inset_polys[1][level_1_wall_idx]);
            if (PolygonUtils::polygonsIntersect(inner, part_outer_wall))
            {
                ++num_level_1_insets;
                part_inner_walls.add(inner[0]);
                // consume the level 1 inset
                inset_polys[1].erase(inset_polys[1].begin() + level_1_wall_idx);
                --level_1_wall_idx; // we've shortened the vector so decrement the index otherwise, we'll skip an element

                // now find all the insets that immediately fill the level 1 inset and consume them also
                Polygons enclosing_insets; // the set of insets that we are trying to "fill in"
                enclosing_insets.add(inner[0]);
                Polygons next_level_enclosing_insets;
                for (unsigned inset_level = 2; inset_level < num_insets && inset_polys[inset_level].size(); ++inset_level)
                {
                    // test the level N insets to see if they are adjacent to any of the level N-1 insets
                    for (unsigned level_n_wall_idx = 0; level_n_wall_idx < inset_polys[inset_level].size(); ++level_n_wall_idx)
                    {
                        for (ConstPolygonRef enclosing_inset : enclosing_insets)
                        {
                            ConstPolygonRef level_n_inset = inset_polys[inset_level][level_n_wall_idx];
                            if (PolygonUtils::polygonOutlinesAdjacent(level_n_inset, enclosing_inset, wall_line_width_x * 1.1f))
                            {
                                next_level_enclosing_insets.add(level_n_inset);
                                part_inner_walls.add(level_n_inset);
                                inset_polys[inset_level].erase(inset_polys[inset_level].begin() + level_n_wall_idx);
                                --level_n_wall_idx; // we've shortened the vector so decrement the index otherwise, we'll skip an element
                                break;
                            }
                        }
                    }
                    enclosing_insets = next_level_enclosing_insets;
                    next_level_enclosing_insets.clear();
                }
            }
        }

        if (part_inner_walls.size() > 0 && extruder_nr == mesh.getSettingAsExtruderNr("wall_x_extruder_nr"))
        {
            gcode_writer.setExtruder_addPrime(storage, gcode_layer, layer_nr, extruder_nr);
            gcode_layer.setIsInside(true); // going to print stuff inside print object
            if (outer_inset_first)
            {
                if (extruder_nr == mesh.getSettingAsExtruderNr("wall_0_extruder_nr"))
                {
                    if (compensate_overlap_0)
                    {
                        WallOverlapComputation wall_overlap_computation(part_outer_wall, wall_line_width_0);
                        gcode_layer.addPolygonsByOptimizer(part_outer_wall, mesh_config.inset0_config, &wall_overlap_computation, z_seam_type, z_seam_pos, wall_0_wipe_dist, spiralize, flow, retract_before_outer_wall);
                    }
                    else
                    {
                        WallOverlapComputation* wall_overlap_computation(nullptr);
                        gcode_layer.addPolygonsByOptimizer(part_outer_wall, mesh_config.inset0_config, wall_overlap_computation, z_seam_type, z_seam_pos, wall_0_wipe_dist, spiralize, flow, retract_before_outer_wall);
                    }
                }
                if (compensate_overlap_x)
                {
                    WallOverlapComputation wall_overlap_computation(part_inner_walls, wall_line_width_x);
                    gcode_layer.addPolygonsByOptimizer(part_inner_walls, mesh_config.insetX_config, &wall_overlap_computation);
                }
                else
                {
                    gcode_layer.addPolygonsByOptimizer(part_inner_walls, mesh_config.insetX_config);
                }
            }
            else
            {
                // just like we did for the holes, ensure that a single outer wall inset is started close to the z seam position
                // but if there is more than one outer wall level 1 inset, don't bother to move as it may actually be a waste of time because
                // there may not be an inset immediately inside of where the z seam is located so we would end up moving again anyway
                if (z_seam_type == EZSeamType::USER_SPECIFIED && num_level_1_insets == 1)
                {
                    // determine the location of the z seam
                    const int z_seam_idx = PolygonUtils::findNearestVert(z_seam_pos, inset_polys[0][0]);
                    const ClosestPolygonPoint z_seam_location(inset_polys[0][0][z_seam_idx], z_seam_idx, inset_polys[0][0]);
                    // move to the location of the vertex in the level 1 inset that's closest to the z seam location
                    const Point dest = part_inner_walls[0][PolygonUtils::findNearestVert(z_seam_location.location, part_inner_walls[0])];
                    gcode_layer.addTravel(dest);
                }
                if (compensate_overlap_x)
                {
                    WallOverlapComputation wall_overlap_computation(part_inner_walls, wall_line_width_x);
                    gcode_layer.addPolygonsByOptimizer(part_inner_walls, mesh_config.insetX_config, &wall_overlap_computation);
                }
                else
                {
                    gcode_layer.addPolygonsByOptimizer(part_inner_walls, mesh_config.insetX_config);
                }
                if (extruder_nr == mesh.getSettingAsExtruderNr("wall_0_extruder_nr"))
                {
                    if (compensate_overlap_0)
                    {
                        WallOverlapComputation wall_overlap_computation(part_outer_wall, wall_line_width_0);
                        gcode_layer.addPolygonsByOptimizer(part_outer_wall, mesh_config.inset0_config, &wall_overlap_computation, z_seam_type, z_seam_pos, wall_0_wipe_dist, spiralize, flow, retract_before_outer_wall);
                    }
                    else
                    {
                        WallOverlapComputation* wall_overlap_computation(nullptr);
                        gcode_layer.addPolygonsByOptimizer(part_outer_wall, mesh_config.inset0_config, wall_overlap_computation, z_seam_type, z_seam_pos, wall_0_wipe_dist, spiralize, flow, retract_before_outer_wall);
                    }
                }
            }
            added_something = true;
        }
        else if (extruder_nr == mesh.getSettingAsExtruderNr("wall_0_extruder_nr"))
        {
            // just the outer wall, no inners

            gcode_writer.setExtruder_addPrime(storage, gcode_layer, layer_nr, extruder_nr);
            gcode_layer.setIsInside(true); // going to print stuff inside print object
            if (compensate_overlap_0)
            {
                WallOverlapComputation wall_overlap_computation(part_outer_wall, wall_line_width_0);
                gcode_layer.addPolygonsByOptimizer(part_outer_wall, mesh_config.inset0_config, &wall_overlap_computation, z_seam_type, z_seam_pos, wall_0_wipe_dist, spiralize, flow, retract_before_outer_wall);
            }
            else
            {
                WallOverlapComputation* wall_overlap_computation(nullptr);
                gcode_layer.addPolygonsByOptimizer(part_outer_wall, mesh_config.inset0_config, wall_overlap_computation, z_seam_type, z_seam_pos, wall_0_wipe_dist, spiralize, flow, retract_before_outer_wall);
            }
            added_something = true;
        }
    }
}

bool InsetOrderOptimizer::processInsetsWithOptimizedOrdering()
{
    added_something = false;
    const unsigned num_insets = part.insets.size();

    // create a vector of vectors containing all the inset polys
    inset_polys.clear();
    for (unsigned inset_level = 0; inset_level < num_insets; ++inset_level)
    {
        inset_polys.emplace_back();
        for (unsigned poly_idx = 0; poly_idx < part.insets[inset_level].size(); ++poly_idx)
        {
            inset_polys[inset_level].push_back(part.insets[inset_level][poly_idx]);
        }
    }

    // first process all the holes and their enclosing insets
    processHoleInsets();

    // then process the part's outer wall and its enclosed insets
    processOuterWallInsets();

    // finally, mop up all the remaining insets that can occur in the gaps between holes
    if (extruder_nr == mesh.getSettingAsExtruderNr("wall_x_extruder_nr"))
    {
        Polygons remaining;
        for (unsigned inset_level = 1; inset_level < inset_polys.size(); ++inset_level)
        {
            const unsigned num_polys = inset_polys[inset_level].size();
            if (inset_level == 1 && num_polys > 0)
            {
                logWarning("Layer %d, %lu level 1 insets remaining to be output (should be 0!)\n", layer_nr, num_polys);
            }
            for (unsigned poly_idx = 0; poly_idx < num_polys; ++poly_idx)
            {
                remaining.add(inset_polys[inset_level][poly_idx]);
            }
        }
        if (remaining.size() > 0)
        {
            gcode_writer.setExtruder_addPrime(storage, gcode_layer, layer_nr, extruder_nr);
            gcode_layer.setIsInside(true); // going to print stuff inside print object
            if (mesh.getSettingBoolean("travel_compensate_overlapping_walls_x_enabled"))
            {
                WallOverlapComputation wall_overlap_computation(remaining, mesh_config.insetX_config.getLineWidth());
                gcode_layer.addPolygonsByOptimizer(remaining, mesh_config.insetX_config, &wall_overlap_computation);
            }
            else
            {
                gcode_layer.addPolygonsByOptimizer(remaining, mesh_config.insetX_config);
            }
            added_something = true;
        }
    }
    return added_something;
}

bool InsetOrderOptimizer::optimizingInsetsIsWorthwhile(const SliceMeshStorage& mesh, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, unsigned int layer_nr, EZSeamType z_seam_type, Point z_seam_pos)
{
    if (!mesh.getSettingBoolean("optimize_wall_printing_order"))
    {
        // optimization disabled
        return false;
    }
    if (mesh.getSettingBoolean("infill_before_walls"))
    {
        // apparently, when printing infill before walls, the insets have to be ordered from infill to outer wall
        // to avoid introducing visible artifacts - the optimiser doesn't guarantee that ordering so don't optimise
        return false;
    }
    const unsigned num_insets = part.insets.size();
    if (num_insets < 2)
    {
        // only 1 inset, definitely not worth optimizing
        return false;
    }
    const unsigned num_holes = part.insets[0].size() - 1;
    if (num_holes == 0)
    {
        if (z_seam_type == EZSeamType::USER_SPECIFIED)
        {
            // will start the inner inset(s) near the z seam location
            return true;
        }
        // no holes, definitely not worth optimizing
        return false;
    }
    if (num_insets > 2)
    {
        // we have holes with three or more insets, good chance it's worth optimizing
        return true;
    }
    if (num_holes == 1)
    {
        if (part.insets[1].size() > 2)
        {
            // there's only 1 hole but more than 2 level 1 insets - it's probably quicker to
            // print without optimization as then all of the level 1 insets will be printed as a group
            return false;
        }
    }
    // for all other cases, the default is to optimize
    return true;
}


}//namespace cura