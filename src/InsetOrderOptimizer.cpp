//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ExtruderTrain.h"
#include "FffGcodeWriter.h"
#include "InsetOrderOptimizer.h"
#include "LayerPlan.h"
#include "wallOverlap.h"
#include "utils/logoutput.h"

namespace cura
{

static int findAdjacentEnclosingPoly(const ConstPolygonRef& enclosed_inset, const std::vector<ConstPolygonPointer>& possible_enclosing_polys, const coord_t max_gap)
{
    // given an inset, search a collection of insets for the adjacent enclosing inset
    for (unsigned int enclosing_poly_idx = 0; enclosing_poly_idx < possible_enclosing_polys.size(); ++enclosing_poly_idx)
    {
        const ConstPolygonRef& enclosing = *possible_enclosing_polys[enclosing_poly_idx];
        // as holes don't overlap, if the insets intersect, it is safe to assume that the enclosed inset is inside the enclosing inset
        if (PolygonUtils::polygonsIntersect(enclosing, enclosed_inset) && PolygonUtils::polygonOutlinesAdjacent(enclosed_inset, enclosing, max_gap))
        {
            return enclosing_poly_idx;
        }
    }
    return -1;
}

void InsetOrderOptimizer::moveInside()
{
    const coord_t outer_wall_line_width = mesh_config.inset0_config.getLineWidth();
    Point p = gcode_layer.getLastPlannedPositionOrStartingPosition();
    // try to move p inside the outer wall by 1.1 times the outer wall line width
    if (PolygonUtils::moveInside(part.insets[0], p, outer_wall_line_width * 1.1f) != NO_INDEX)
    {
        if (!retraction_region_calculated)
        {
            retraction_region = part.insets[0].offset(-outer_wall_line_width);
            retraction_region_calculated = true;
        }
        // move to p if it is not closer than a line width from the centre line of the outer wall
        if (retraction_region.inside(p))
        {
            gcode_layer.addTravel_simple(p);
            gcode_layer.forceNewPathStart();
        }
        else
        {
            // p is still too close to the centre line of the outer wall so move it again
            // this can occur when the last wall finished at a right angle corner as the first move
            // just moved p along one edge rather than into the part
            if (PolygonUtils::moveInside(part.insets[0], p, outer_wall_line_width * 1.1f) != NO_INDEX)
            {
                // move to p if it is not closer than a line width from the centre line of the outer wall
                if (retraction_region.inside(p))
                {
                    gcode_layer.addTravel_simple(p);
                    gcode_layer.forceNewPathStart();
                }
            }
        }
    }
}

void InsetOrderOptimizer::processHoleInsets()
{
    if (inset_polys[0].size() < 2)
    {
        // part has no holes - the code below has a problem in that it causes the z-seams not
        // to be aligned in the specific situation where infill was being printed before walls, the
        // inner walls were printed before the outer and the part has no holes!
        return;
    }

    const coord_t wall_line_width_0 = mesh_config.inset0_config.getLineWidth();
    const coord_t wall_line_width_x = mesh_config.insetX_config.getLineWidth();
    const coord_t max_gap = std::max(wall_line_width_0, wall_line_width_x) * 1.1f; // if polys are closer than this, they are considered adjacent
    const coord_t wall_0_wipe_dist = mesh.settings.get<coord_t>("wall_0_wipe_dist");
    const bool retract_before_outer_wall = mesh.settings.get<bool>("travel_retract_before_outer_wall");
    const bool outer_inset_first = mesh.settings.get<bool>("outer_inset_first")
        || (layer_nr == 0 && mesh.settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::BRIM);
    const unsigned int num_insets = part.insets.size();
    constexpr float flow = 1.0;

    if (!outer_inset_first && mesh.settings.get<bool>("infill_before_walls"))
    {
        // special case when infill is output before walls and walls are being printed inside to outside
        // we need to ensure that the insets are output in order, innermost first
        // so detect any higher level insets that don't surround holes and output them before the insets that do surround holes
        for (unsigned inset_level = num_insets - 1; inset_level > 0; --inset_level)
        {
            Polygons insets_that_do_not_surround_holes;
            for (unsigned inset_idx = 0; inset_idx < inset_polys[0].size() && inset_idx < inset_polys[inset_level].size(); ++inset_idx)
            {
                const ConstPolygonRef& inner_wall = *inset_polys[inset_level][inset_idx];
                const ConstPolygonRef& outer_wall = *inset_polys[0][inset_idx];
                // little subtlety here, don't test first inset against inset_polys[0][0] as it will always intersect
                bool inset_surrounds_hole = inset_idx > 0 && PolygonUtils::polygonsIntersect(inner_wall, outer_wall);
                if (!inset_surrounds_hole)
                {
                    // the inset didn't surround the level 0 inset with the same inset_idx but maybe it surrounds another hole
                    // start this loop at 1 not 0 as everything is surrounded by the part outline!
                    for (unsigned hole_idx = 1; !inset_surrounds_hole && hole_idx < inset_polys[0].size(); ++hole_idx)
                    {
                        const ConstPolygonRef& outer_wall = *inset_polys[0][hole_idx];
                        inset_surrounds_hole = PolygonUtils::polygonsIntersect(inner_wall, outer_wall);
                    }
                }
                if (!inset_surrounds_hole)
                {
                    // consume this inset
                    insets_that_do_not_surround_holes.add(inner_wall);
                    inset_polys[inset_level].erase(inset_polys[inset_level].begin() + inset_idx);
                    --inset_idx; // we've shortened the vector so decrement the index otherwise, we'll skip an element
                }
            }
            if (insets_that_do_not_surround_holes.size() > 0 && extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr)
            {
                gcode_writer.setExtruder_addPrime(storage, gcode_layer, extruder_nr);
                gcode_layer.setIsInside(true); // going to print stuff inside print object
                gcode_layer.addWalls(insets_that_do_not_surround_holes, mesh, mesh_config.insetX_config, mesh_config.bridge_insetX_config, wall_overlapper_x);
                added_something = true;
            }
        }
    }

    // work out the order we wish to visit all the holes

    // if the z-seam location on the part's outline can be determined here, optimize to minimize the distance travelled from the last hole
    // to the z-seam - one possible benefit of this strategy is that by minimizing the distance travelled to the outline, the accuracy of
    // that movement is potentially improved (less overshoot, backlash, etc.) which could make a visible difference (especially if the
    // outer wall is printed first).

    // if we can't determine here where the z-seam on the outline will be, optimize to minimize the distance travelled from the current location
    // to the first hole

    Point start_point = gcode_layer.getLastPlannedPositionOrStartingPosition(); // where we are now
    const bool optimize_backwards = (z_seam_config.type == EZSeamType::USER_SPECIFIED || z_seam_config.type == EZSeamType::SHARPEST_CORNER);
    if (optimize_backwards)
    {
        // determine the location of the z-seam and use that as the start point
        PathOrderOptimizer order_optimizer(Point(), z_seam_config);
        order_optimizer.addPolygon(*inset_polys[0][0]);
        order_optimizer.optimize();
        const unsigned outer_poly_start_idx = gcode_layer.locateFirstSupportedVertex(*inset_polys[0][0], order_optimizer.polyStart[0]);
        start_point = (*inset_polys[0][0])[outer_poly_start_idx];
    }
    Polygons comb_boundary(*gcode_layer.getCombBoundaryInside());
    comb_boundary.simplify(MM2INT(0.1), MM2INT(0.1));
    PathOrderOptimizer order_optimizer(start_point, z_seam_config, &comb_boundary);
    for (unsigned int poly_idx = 1; poly_idx < inset_polys[0].size(); poly_idx++)
    {
        order_optimizer.addPolygon(*inset_polys[0][poly_idx]);
    }
    order_optimizer.optimize();
    if (optimize_backwards)
    {
        // reverse the optimized order so we end up as near to the outline z-seam as possible
        std::reverse(order_optimizer.polyOrder.begin(), order_optimizer.polyOrder.end());
    }

    // this will consume all of the insets that surround holes but not the insets next to the outermost wall of the model
    for (unsigned int outer_poly_order_idx = 0; outer_poly_order_idx < order_optimizer.polyOrder.size(); ++outer_poly_order_idx)
    {
        Polygons hole_outer_wall; // the outermost wall of a hole
        hole_outer_wall.add(*inset_polys[0][order_optimizer.polyOrder[outer_poly_order_idx] + 1]); // +1 because first element (part outer wall) wasn't included
        std::vector<unsigned int> hole_level_1_wall_indices; // the indices of the walls that touch the hole's outer wall
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
                    inset.add(*inset_polys[1][adjacent_enclosing_poly_idx]);
                    int num_future_outlines_touched = 0; // number of outlines that have yet to be output that are touched by this level 1 inset
                    // does it touch the outer wall?
                    if (PolygonUtils::polygonOutlinesAdjacent(inset[0], *inset_polys[0][0], max_gap))
                    {
                        // yes, the level 1 inset touches the part's outer wall
                        ++num_future_outlines_touched;
                    }
                    // does it touch any yet to be processed hole outlines?
                    for (unsigned int order_index = outer_poly_order_idx + 1; num_future_outlines_touched < 1 && order_index < order_optimizer.polyOrder.size(); ++order_index)
                    {
                        int outline_index = order_optimizer.polyOrder[order_index] + 1; // +1 because first element (part outer wall) wasn't included
                        // as we don't know the shape of the outlines (straight, concave, convex, etc.) and the
                        // adjacency test assumes that the poly's are arranged so that the first has smaller
                        // radius curves than the second (it's "inside" the second) we need to test both combinations
                        if (PolygonUtils::polygonOutlinesAdjacent(inset[0], *inset_polys[0][outline_index], max_gap) ||
                            PolygonUtils::polygonOutlinesAdjacent(*inset_polys[0][outline_index], inset[0], max_gap))
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

        for (unsigned int level_1_wall_idx = 0; level_1_wall_idx < hole_level_1_wall_indices.size(); ++level_1_wall_idx)
        {
            // add the level 1 inset to the collection of inner walls to be printed and consume it, taking care to adjust
            // those elements in hole_level_1_wall_indices that are larger
            unsigned int inset_idx = hole_level_1_wall_indices[level_1_wall_idx];
            ConstPolygonPointer last_inset = inset_polys[1][inset_idx];
            hole_inner_walls.add(ConstPolygonRef(*last_inset));
            inset_polys[1].erase(inset_polys[1].begin() + inset_idx);
            // decrement any other indices in hole_level_1_wall_indices that are greater than inset_idx
            for (unsigned int i = level_1_wall_idx + 1; i < hole_level_1_wall_indices.size(); ++i)
            {
                if (hole_level_1_wall_indices[i] > inset_idx)
                {
                    hole_level_1_wall_indices[i] = hole_level_1_wall_indices[i] - 1;
                }
            }
            // now find all the insets that immediately surround the level 1 wall and consume them
            for (unsigned int inset_level = 2; inset_level < num_insets && inset_polys[inset_level].size(); ++inset_level)
            {
                int i = findAdjacentEnclosingPoly(ConstPolygonRef(*last_inset), inset_polys[inset_level], wall_line_width_x * 1.1f);
                if (i >= 0)
                {
                    // we have found an enclosing inset
                    if (outer_inset_first)
                    {
                        // when printing outer insets first we don't want to print this enclosing inset
                        // if it also encloses other holes that haven't yet been processed so check the holes
                        // that haven't yet been processed to see if they are also enclosed by this enclosing inset
                        const ConstPolygonRef& enclosing_inset = *inset_polys[inset_level][i];
                        bool encloses_future_hole = false; // set true if this inset also encloses another hole that hasn't yet been processed
                        for (unsigned int hole_order_index = outer_poly_order_idx + 1; !encloses_future_hole && hole_order_index < order_optimizer.polyOrder.size(); ++hole_order_index)
                        {
                            const ConstPolygonRef& enclosed_inset = *inset_polys[0][order_optimizer.polyOrder[hole_order_index] + 1]; // +1 because first element (part outer wall) wasn't included
                            encloses_future_hole = PolygonUtils::polygonsIntersect(enclosing_inset, enclosed_inset);
                        }
                        if (encloses_future_hole)
                        {
                            // give up finding the insets that surround this hole
                            break;
                        }
                    }
                    // it's OK to print the enclosing inset and see if any further enclosing insets can also be printed
                    last_inset = inset_polys[inset_level][i];
                    hole_inner_walls.add(ConstPolygonRef(*last_inset));
                    inset_polys[inset_level].erase(inset_polys[inset_level].begin() + i);
                }
            }
        }

        if (hole_inner_walls.size() > 0 && extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr)
        {
            // output the inset polys

            gcode_writer.setExtruder_addPrime(storage, gcode_layer, extruder_nr);
            gcode_layer.setIsInside(true); // going to print stuff inside print object
            if (outer_inset_first)
            {
                if (extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr)
                {
                    gcode_layer.addWalls(hole_outer_wall, mesh, mesh_config.inset0_config, mesh_config.bridge_inset0_config, wall_overlapper_0, z_seam_config, wall_0_wipe_dist, flow, retract_before_outer_wall);
                }
                gcode_layer.addWalls(hole_inner_walls, mesh, mesh_config.insetX_config, mesh_config.bridge_insetX_config, wall_overlapper_x);
            }
            else
            {
                // we want the insets that surround the hole to start as close as possible to the z seam location so to
                // avoid the possible retract when moving from the end of the immediately enclosing inset to the start
                // of the hole outer wall we first move to a location that is close to the z seam and at a vertex of the
                // first inset we want to be printed
                const unsigned outer_poly_idx = order_optimizer.polyOrder[outer_poly_order_idx];
                unsigned outer_poly_start_idx = gcode_layer.locateFirstSupportedVertex(hole_outer_wall[0], order_optimizer.polyStart[outer_poly_idx]);

                // detect special case where the z-seam is located on the sharpest corner and there is only 1 hole and
                // the gap between the walls is just a few line widths
                if (z_seam_config.type == EZSeamType::SHARPEST_CORNER && inset_polys[0].size() == 2 && PolygonUtils::polygonOutlinesAdjacent(*inset_polys[0][1], *inset_polys[0][0], max_gap * 4))
                {
                    // align z-seam of hole with z-seam of outer wall - makes a nicer job when printing tubes
                    outer_poly_start_idx = PolygonUtils::findNearestVert(start_point, hole_outer_wall.back());
                }
                const Point z_seam_location = hole_outer_wall[0][outer_poly_start_idx];
                // move to the location of the vertex in the outermost enclosing inset that's closest to the z seam location
                const Point dest = hole_inner_walls.back()[PolygonUtils::findNearestVert(z_seam_location, hole_inner_walls.back())];
                gcode_layer.addTravel(dest);
                std::reverse(hole_inner_walls.begin(), hole_inner_walls.end());
                gcode_layer.addWalls(hole_inner_walls, mesh, mesh_config.insetX_config, mesh_config.bridge_insetX_config, wall_overlapper_x);
                if (extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr)
                {
                    gcode_layer.addWall(hole_outer_wall[0], outer_poly_start_idx, mesh, mesh_config.inset0_config, mesh_config.bridge_inset0_config, wall_overlapper_0, wall_0_wipe_dist, flow, retract_before_outer_wall);
                    // move inside so an immediately following retract doesn't occur on the outer wall
                    moveInside();
                }
            }
            added_something = true;
        }
        else if (extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr)
        {
            // just the outer wall, no level 1 insets
            gcode_writer.setExtruder_addPrime(storage, gcode_layer, extruder_nr);
            gcode_layer.setIsInside(true); // going to print stuff inside print object

            // detect special case where where the z-seam is located on the sharpest corner and there is only 1 hole and
            // the gap between the walls is just a few line widths
            if (z_seam_config.type == EZSeamType::SHARPEST_CORNER && inset_polys[0].size() == 2 && PolygonUtils::polygonOutlinesAdjacent(*inset_polys[0][1], *inset_polys[0][0], max_gap * 2))
            {
                // align z-seam of hole with z-seam of outer wall - makes a nicer job when printing tubes
                const unsigned point_idx = PolygonUtils::findNearestVert(start_point, hole_outer_wall.back());
                gcode_layer.addWall(hole_outer_wall.back(), point_idx, mesh, mesh_config.inset0_config, mesh_config.bridge_inset0_config, wall_overlapper_0, wall_0_wipe_dist, flow, retract_before_outer_wall);
            }
            else
            {
                gcode_layer.addWalls(hole_outer_wall, mesh, mesh_config.inset0_config, mesh_config.bridge_inset0_config, wall_overlapper_0, z_seam_config, wall_0_wipe_dist, flow, retract_before_outer_wall);
            }
            // move inside so an immediately following retract doesn't occur on the outer wall
            moveInside();
            added_something = true;
        }
    }
}

void InsetOrderOptimizer::processOuterWallInsets(const bool include_outer, const bool include_inners)
{
    const coord_t wall_line_width_x = mesh_config.insetX_config.getLineWidth();
    const coord_t wall_0_wipe_dist = mesh.settings.get<coord_t>("wall_0_wipe_dist");
    const bool retract_before_outer_wall = mesh.settings.get<bool>("travel_retract_before_outer_wall");
    const bool outer_inset_first = mesh.settings.get<bool>("outer_inset_first")
                                    || (layer_nr == 0 && mesh.settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::BRIM);
    const size_t num_insets = part.insets.size();
    constexpr Ratio flow = 1.0_r;

    // process the part's outer wall and the level 1 insets that it surrounds

    ConstPolygonRef outer_wall = *inset_polys[0][0];
    Polygons part_inner_walls;

    if (include_inners)
    {
        // find the level 1 insets that are inside the outer wall and consume them
        for (unsigned int level_1_wall_idx = 0; inset_polys.size() > 1 && level_1_wall_idx < inset_polys[1].size(); ++level_1_wall_idx)
        {
            ConstPolygonRef inner = *inset_polys[1][level_1_wall_idx];
            if (PolygonUtils::polygonsIntersect(inner, outer_wall))
            {
                part_inner_walls.add(inner);
                // consume the level 1 inset
                inset_polys[1].erase(inset_polys[1].begin() + level_1_wall_idx);
                --level_1_wall_idx; // we've shortened the vector so decrement the index otherwise, we'll skip an element

                // now find all the insets that immediately fill the level 1 inset and consume them also
                Polygons enclosing_insets; // the set of insets that we are trying to "fill in"
                enclosing_insets.add(inner);
                Polygons next_level_enclosing_insets;
                for (unsigned int inset_level = 2; inset_level < num_insets && inset_polys[inset_level].size(); ++inset_level)
                {
                    // test the level N insets to see if they are adjacent to any of the level N-1 insets
                    for (unsigned int level_n_wall_idx = 0; level_n_wall_idx < inset_polys[inset_level].size(); ++level_n_wall_idx)
                    {
                        for (ConstPolygonRef enclosing_inset : enclosing_insets)
                        {
                            ConstPolygonRef level_n_inset = *inset_polys[inset_level][level_n_wall_idx];
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
    }

    if (part_inner_walls.size() > 0 && extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr)
    {
        gcode_writer.setExtruder_addPrime(storage, gcode_layer, extruder_nr);
        gcode_layer.setIsInside(true); // going to print stuff inside print object
        // determine the location of the z seam
        PathOrderOptimizer order_optimizer(gcode_layer.getLastPlannedPositionOrStartingPosition(), z_seam_config);
        order_optimizer.addPolygon(*inset_polys[0][0]);
        order_optimizer.optimize();
        const unsigned outer_poly_start_idx = gcode_layer.locateFirstSupportedVertex(*inset_polys[0][0], order_optimizer.polyStart[0]);
        const Point z_seam_location = (*inset_polys[0][0])[outer_poly_start_idx];

        std::function<void(void)> addInnerWalls = [this, part_inner_walls, outer_inset_first, z_seam_location]()
        {
            Polygons boundary(*gcode_layer.getCombBoundaryInside());
            ZSeamConfig inner_walls_z_seam_config;
            PathOrderOptimizer orderOptimizer(z_seam_location, inner_walls_z_seam_config, &boundary);
            orderOptimizer.addPolygons(part_inner_walls);
            orderOptimizer.optimize();
            if (!outer_inset_first)
            {
                // reverse the optimized order so we end up as near to the outline z-seam as possible
                std::reverse(orderOptimizer.polyOrder.begin(), orderOptimizer.polyOrder.end());
            }
            constexpr coord_t wall_0_wipe_dist = 0;
            constexpr float flow_ratio = 1.0;
            constexpr bool always_retract = false;
            for (unsigned int wall_idx : orderOptimizer.polyOrder)
            {
                gcode_layer.addWall(part_inner_walls[wall_idx], orderOptimizer.polyStart[wall_idx], mesh, mesh_config.insetX_config, mesh_config.bridge_insetX_config, wall_overlapper_x, wall_0_wipe_dist, flow_ratio, always_retract);
            }
        };

        if (outer_inset_first)
        {
            if (include_outer && extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr)
            {
                gcode_layer.addWall(*inset_polys[0][0], outer_poly_start_idx, mesh, mesh_config.inset0_config, mesh_config.bridge_inset0_config, wall_overlapper_0, wall_0_wipe_dist, flow, retract_before_outer_wall);
            }
            addInnerWalls();
        }
        else
        {
            addInnerWalls();
            if (include_outer && extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr)
            {
                gcode_layer.addWall(*inset_polys[0][0], outer_poly_start_idx, mesh, mesh_config.inset0_config, mesh_config.bridge_inset0_config, wall_overlapper_0, wall_0_wipe_dist, flow, retract_before_outer_wall);
                // move inside so an immediately following retract doesn't occur on the outer wall
                moveInside();
            }
        }
        added_something = true;
    }
    else if (include_outer && extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr)
    {
        // just the outer wall, no inners

        gcode_writer.setExtruder_addPrime(storage, gcode_layer, extruder_nr);
        gcode_layer.setIsInside(true); // going to print stuff inside print object
        Polygons part_outer_wall;
        part_outer_wall.add(*inset_polys[0][0]);
        gcode_layer.addWalls(part_outer_wall, mesh, mesh_config.inset0_config, mesh_config.bridge_inset0_config, wall_overlapper_0, z_seam_config, wall_0_wipe_dist, flow, retract_before_outer_wall);
        // move inside so an immediately following retract doesn't occur on the outer wall
        moveInside();
        added_something = true;
    }
}

bool InsetOrderOptimizer::processInsetsWithOptimizedOrdering()
{
    added_something = false;
    const unsigned int num_insets = part.insets.size();

    // if overlap compensation is enabled, gather all the level 0 and/or level X walls together
    // and initialise the respective overlap computers
    // NOTE: this code assumes that the overlap computers do not alter the order or number of the polys!
    Polygons wall_0_polys;
    if (mesh.settings.get<bool>("travel_compensate_overlapping_walls_0_enabled"))
    {
        wall_0_polys = part.insets[0];
        wall_overlapper_0 = new WallOverlapComputation(wall_0_polys, mesh_config.inset0_config.getLineWidth());
    }

    Polygons wall_x_polys;
    if (mesh.settings.get<bool>("travel_compensate_overlapping_walls_x_enabled"))
    {
        for (unsigned int inset_level = 1; inset_level < num_insets; ++inset_level)
        {
            wall_x_polys.add(part.insets[inset_level]);
        }
        // use a slightly reduced line width so that compensation only occurs between insets at the same level (and not between insets in adjacent levels)
        wall_overlapper_x = new WallOverlapComputation(wall_x_polys, mesh_config.insetX_config.getLineWidth() - 1);
    }

    // create a vector of vectors containing all the inset polys
    inset_polys.clear();

    // if overlap compensation is enabled, use the polys that have been tweaked by the
    // overlap computers, otherwise use the original, un-compensated, polys

    inset_polys.emplace_back();
    for (unsigned int poly_idx = 0; poly_idx < part.insets[0].size(); ++poly_idx)
    {
        if (wall_overlapper_0)
        {
            inset_polys[0].push_back(wall_0_polys[poly_idx]);
        }
        else
        {
            inset_polys[0].push_back(part.insets[0][poly_idx]);
        }
    }

    unsigned int wall_x_polys_index = 0;
    for (unsigned int inset_level = 1; inset_level < num_insets; ++inset_level)
    {
        inset_polys.emplace_back();
        for (unsigned int poly_idx = 0; poly_idx < part.insets[inset_level].size(); ++poly_idx)
        {
            if (wall_overlapper_x)
            {
                inset_polys[inset_level].push_back(wall_x_polys[wall_x_polys_index++]);
            }
            else
            {
                inset_polys[inset_level].push_back(part.insets[inset_level][poly_idx]);
            }
        }
    }

    // if the print has thin walls due to the distance from a hole to the outer wall being smaller than a line width, it will produce a nicer finish on
    // the outer wall if it is printed before the holes because the outer wall does not get flow reduced but the hole walls will get flow reduced where
    // they are close to the outer wall. However, we only want to do this if the level 0 insets are being printed before the higher level insets.

    if (mesh.settings.get<bool>("outer_inset_first") || (layer_nr == 0 && mesh.settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::BRIM))
    {
        // first process the outer wall only
        processOuterWallInsets(true, false);

        // then process all the holes and their enclosing insets
        processHoleInsets();

        // finally, process the insets enclosed by the part's outer wall
        processOuterWallInsets(false, true);
    }
    else
    {
        // first process all the holes and their enclosing insets
        processHoleInsets();

        // then process the part's outer wall and its enclosed insets
        processOuterWallInsets(true, true);
    }

    // finally, mop up all the remaining insets that can occur in the gaps between holes
    if (extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr)
    {
        Polygons remaining;
        for (unsigned int inset_level = 1; inset_level < inset_polys.size(); ++inset_level)
        {
            const unsigned int num_polys = inset_polys[inset_level].size();
            if (inset_level == 1 && num_polys > 0)
            {
                logWarning("Layer %d, %lu level 1 insets remaining to be output (should be 0!)\n", layer_nr, num_polys);
            }
            for (unsigned int poly_idx = 0; poly_idx < num_polys; ++poly_idx)
            {
                remaining.add(*inset_polys[inset_level][poly_idx]);
            }
        }
        if (remaining.size() > 0)
        {
            gcode_writer.setExtruder_addPrime(storage, gcode_layer, extruder_nr);
            gcode_layer.setIsInside(true); // going to print stuff inside print object
            gcode_layer.addWalls(remaining, mesh, mesh_config.insetX_config, mesh_config.bridge_insetX_config, wall_overlapper_x);
            added_something = true;
        }
    }
    if (wall_overlapper_0)
    {
        delete wall_overlapper_0;
        wall_overlapper_0 = nullptr;
    }
    if (wall_overlapper_x)
    {
        delete wall_overlapper_x;
        wall_overlapper_x = nullptr;
    }
    return added_something;
}

bool InsetOrderOptimizer::optimizingInsetsIsWorthwhile(const SliceMeshStorage& mesh, const SliceLayerPart& part)
{
    if (!mesh.settings.get<bool>("optimize_wall_printing_order"))
    {
        // optimization disabled
        return false;
    }
    if (part.insets.size() == 0)
    {
        // no outlines at all, definitely not worth optimizing
        return false;
    }
    if (part.insets.size() < 2 && part.insets[0].size() < 2)
    {
        // only a single outline and no holes, definitely not worth optimizing
        return false;
    }
    // optimize all other combinations of walls and holes
    return true;
}


}//namespace cura
