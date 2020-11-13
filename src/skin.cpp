//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <cmath> // std::ceil

#include "Application.h" //To get settings.
#include "ExtruderTrain.h"
#include "skin.h"
#include "sliceDataStorage.h"
#include "settings/EnumSettings.h" //For EFillMethod.
#include "settings/types/AngleRadians.h" //For the infill support angle.
#include "settings/types/Ratio.h"
#include "utils/math.h"
#include "utils/polygonUtils.h"
#include "WallToolPaths.h"

#define MIN_AREA_SIZE (0.4 * 0.4)

namespace cura
{

coord_t SkinInfillAreaComputation::getSkinLineWidth(const SliceMeshStorage& mesh, const LayerIndex& layer_nr)
{
    coord_t skin_line_width = mesh.settings.get<coord_t>("skin_line_width");
    if (layer_nr == 0)
    {
        const ExtruderTrain& train_skin = mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr");
        skin_line_width *= train_skin.settings.get<Ratio>("initial_layer_line_width_factor");
    }
    return skin_line_width;
}

coord_t SkinInfillAreaComputation::getWallLineWidth0(const SliceMeshStorage& mesh, const LayerIndex& layer_nr)
{
    coord_t wall_line_width_0 = mesh.settings.get<coord_t>("wall_line_width_0");
    if (layer_nr == 0)
    {
        const ExtruderTrain& train_wall_0 = mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr");
        wall_line_width_0 *= train_wall_0.settings.get<Ratio>("initial_layer_line_width_factor");
    }
    return wall_line_width_0;
}

coord_t SkinInfillAreaComputation::getWallLineWidthX(const SliceMeshStorage& mesh, const LayerIndex& layer_nr)
{
    coord_t wall_line_width_x = mesh.settings.get<coord_t>("wall_line_width_x");
    if (layer_nr == 0)
    {
        const ExtruderTrain& train_wall_x = mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr");
        wall_line_width_x *= train_wall_x.settings.get<Ratio>("initial_layer_line_width_factor");
    }
    return wall_line_width_x;
}

SkinInfillAreaComputation::SkinInfillAreaComputation(const LayerIndex& layer_nr, SliceMeshStorage& mesh, bool process_infill)
: layer_nr(layer_nr)
, mesh(mesh)
, bottom_layer_count(mesh.settings.get<size_t>("bottom_layers"))
, initial_bottom_layer_count(mesh.settings.get<size_t>("initial_bottom_layers"))
, top_layer_count(mesh.settings.get<size_t>("top_layers"))
, wall_line_count(mesh.settings.get<size_t>("wall_line_count"))
, skin_line_width(getSkinLineWidth(mesh, layer_nr))
, wall_line_width_0(getWallLineWidth0(mesh, layer_nr))
, wall_line_width_x(getWallLineWidthX(mesh, layer_nr))
, innermost_wall_line_width((wall_line_count == 1) ? wall_line_width_0 : wall_line_width_x)
, skin_inset_count(mesh.settings.get<size_t>("skin_outline_count"))
, no_small_gaps_heuristic(mesh.settings.get<bool>("skin_no_small_gaps_heuristic"))
, process_infill(process_infill)
, top_skin_preshrink(mesh.settings.get<coord_t>("top_skin_preshrink"))
, bottom_skin_preshrink(mesh.settings.get<coord_t>("bottom_skin_preshrink"))
, top_skin_expand_distance(mesh.settings.get<coord_t>("top_skin_expand_distance"))
, bottom_skin_expand_distance(mesh.settings.get<coord_t>("bottom_skin_expand_distance"))
{
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * this function may only read/write the skin and infill from the *current* layer.
 */
Polygons SkinInfillAreaComputation::getOutlineOnLayer(const SliceLayerPart& part_here, const LayerIndex layer2_nr)
{
    Polygons result;
    if (layer2_nr >= static_cast<int>(mesh.layers.size()))
    {
        return result;
    }
    const SliceLayer& layer2 = mesh.layers[layer2_nr];
    for (const SliceLayerPart& part2 : layer2.parts)
    {
        if (part_here.boundaryBox.hit(part2.boundaryBox))
        {
            result.add(part2.outline);
        }
    }
    return result;
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * generateSkinAreas reads data from mesh.layers.parts[*].insets and writes to mesh.layers[n].parts[*].skin_parts
 * generateSkinInsets only read/writes the skin_parts from the current layer.
 *
 * generateSkins therefore reads (depends on) data from mesh.layers[*].parts[*].insets and writes mesh.layers[n].parts[*].skin_parts
 */
void SkinInfillAreaComputation::generateSkinsAndInfill()
{
    generateSkinAndInfillAreas();

    SliceLayer* layer = &mesh.layers[layer_nr];
    for (unsigned int part_nr = 0; part_nr < layer->parts.size(); part_nr++)
    {
        SliceLayerPart& part = layer->parts[part_nr];
        generateSkinInsetsAndInnerSkinInfill(&part);

        generateRoofing(part);
    }
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * generateSkinAreas reads data from mesh.layers[*].parts[*].insets and writes to mesh.layers[n].parts[*].skin_parts
 */
void SkinInfillAreaComputation::generateSkinAndInfillAreas()
{
    SliceLayer& layer = mesh.layers[layer_nr];

    if (!process_infill && bottom_layer_count == 0 && top_layer_count == 0)
    {
        return;
    }

    for(SliceLayerPart& part : layer.parts)
    {
        generateSkinAndInfillAreas(part);
    }
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * generateSkinAreas reads data from mesh.layers[*].parts[*].insets and writes to mesh.layers[n].parts[*].skin_parts
 */
void SkinInfillAreaComputation::generateSkinAndInfillAreas(SliceLayerPart& part)
{
    //Make a copy of the outline which we later intersect and union with the resized skins to ensure the resized skin isn't too large or removed completely.
    Polygons top_skin;
    if (top_layer_count > 0)
    {
        top_skin = Polygons(part.inner_area);
    }
    Polygons bottom_skin;
    if (bottom_layer_count > 0 || layer_nr < LayerIndex(initial_bottom_layer_count))
    {
        bottom_skin = Polygons(part.inner_area);
    }

    calculateBottomSkin(part, bottom_skin);
    calculateTopSkin(part, top_skin);

    applySkinExpansion(part.inner_area, top_skin, bottom_skin);

    //Now combine the resized top skin and bottom skin.
    Polygons skin = top_skin.unionPolygons(bottom_skin);

    skin.removeSmallAreas(MIN_AREA_SIZE);

    if (process_infill)
    { // process infill when infill density > 0
        // or when other infill meshes want to modify this infill
        generateInfill(part, skin);
    }

    for (PolygonsPart& skin_area_part : skin.splitIntoParts())
    {
        part.skin_parts.emplace_back();
        part.skin_parts.back().outline = skin_area_part;
    }
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * this function may only read/write the skin and infill from the *current* layer.
 */
void SkinInfillAreaComputation::calculateBottomSkin(const SliceLayerPart& part, Polygons& downskin)
{
    if (bottom_layer_count == 0 && initial_bottom_layer_count == 0)
    {
        return; // downskin remains empty
    }
    if (layer_nr < LayerIndex(initial_bottom_layer_count))
    {
        return; // don't subtract anything form the downskin
    }
    LayerIndex bottom_check_start_layer_idx = std::max(LayerIndex(0), layer_nr - bottom_layer_count);
    Polygons not_air = getOutlineOnLayer(part, bottom_check_start_layer_idx);
    if (!no_small_gaps_heuristic)
    {
        for (int downskin_layer_nr = bottom_check_start_layer_idx + 1; downskin_layer_nr < layer_nr; downskin_layer_nr++)
        {
            not_air = not_air.intersection(getOutlineOnLayer(part, downskin_layer_nr));
        }
    }
    const double min_infill_area = mesh.settings.get<double>("min_infill_area");
    if (min_infill_area > 0.0)
    {
        not_air.removeSmallAreas(min_infill_area);
    }
    downskin = downskin.difference(not_air); // skin overlaps with the walls
}

void SkinInfillAreaComputation::calculateTopSkin(const SliceLayerPart& part, Polygons& upskin)
{
    if(layer_nr > LayerIndex(mesh.layers.size()) - top_layer_count || top_layer_count <= 0)
    {
        //If we're in the very top layers (less than top_layer_count from the top of the mesh) everything will be top skin anyway, so no need to generate infill. Just take the original inner contour.
        //If top_layer_count is 0, no need to calculate anything either.
        return;
    }

    Polygons not_air = getOutlineOnLayer(part, layer_nr + top_layer_count);
    if (!no_small_gaps_heuristic)
    {
        for (int upskin_layer_nr = layer_nr + 1; upskin_layer_nr < layer_nr + top_layer_count; upskin_layer_nr++)
        {
            not_air = not_air.intersection(getOutlineOnLayer(part, upskin_layer_nr));
        }
    }

    const double min_infill_area = mesh.settings.get<double>("min_infill_area");
    if (min_infill_area > 0.0)
    {
        not_air.removeSmallAreas(min_infill_area);
    }

    upskin = upskin.difference(not_air); // skin overlaps with the walls
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * this function may only read/write the skin and infill from the *current* layer.
 */
void SkinInfillAreaComputation::applySkinExpansion(const Polygons& original_outline, Polygons& upskin, Polygons& downskin)
{
    //Remove thin pieces of support for Skin Removal Width.
    if(bottom_skin_preshrink)
    {
        downskin = downskin.offset(-bottom_skin_preshrink / 2).offset(bottom_skin_preshrink / 2);
    }
    if(top_skin_preshrink)
    {
        upskin = upskin.offset(-top_skin_preshrink / 2).offset(top_skin_preshrink / 2);
    }

    //Expand some areas of the skin for Skin Expand Distance.
    const coord_t min_width = mesh.settings.get<coord_t>("min_skin_width_for_expansion") / 2;
    if(min_width)
    {
        //This performs an opening operation by first insetting by the minimum width, then offsetting with the same width.
        //The expansion is only applied to that opened shape.
        if(bottom_skin_expand_distance)
        {
            const Polygons expanded = downskin.offset(-min_width).offset(min_width + bottom_skin_expand_distance);
            //And then re-joined with the original part that was not offset, to retain parts smaller than min_width.
            downskin = downskin.unionPolygons(expanded);
            //Afterwards the offset shape needs to be clipped with the original outline since we may only expand into infill, not across the walls.
            downskin = downskin.intersection(original_outline);
        }
        if(top_skin_expand_distance)
        {
            const Polygons expanded = upskin.offset(-min_width).offset(min_width + top_skin_expand_distance);
            upskin = upskin.unionPolygons(expanded);
            upskin = upskin.intersection(original_outline);
        }
    }
    else
    {
        //Without minimum width, it's just a simple offset. No opening operation or re-join necessary.
        if(bottom_skin_expand_distance)
        {
            downskin = downskin.offset(bottom_skin_expand_distance);
            //Afterwards the offset shape needs to be clipped with the original outline since we may only expand into infill, not across the walls.
            downskin = downskin.intersection(original_outline);
        }
        if(top_skin_expand_distance)
        {
            upskin = upskin.offset(top_skin_expand_distance);
            upskin = upskin.intersection(original_outline);
        }
    }

}


/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * this function may only read/write the skin and infill from the *current* layer.
 */
void SkinInfillAreaComputation::generateSkinInsetsAndInnerSkinInfill(SliceLayerPart* part)
{
    for (SkinPart& skin_part : part->skin_parts)
    {
        // Do not generate skinfill if the Bottom Pattern Initial Layer (for layer 0) and the Top/Bottom Layer Pattern
        // (for the rest of the layers) are concentric
        const bool concentric_skinfill_patern =
               (layer_nr == 0 && mesh.settings.get<EFillMethod>("top_bottom_pattern_0") == EFillMethod::CONCENTRIC)
            || (layer_nr > 0  && mesh.settings.get<EFillMethod>("top_bottom_pattern") == EFillMethod::CONCENTRIC);
        generateSkinInsets(skin_part, concentric_skinfill_patern);
    }
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * this function may only read/write the skin and infill from the *current* layer.
 */
void SkinInfillAreaComputation::generateSkinInsets(SkinPart& skin_part, const bool concentric_skinfill_patern)
{
    if (skin_inset_count > 0 || concentric_skinfill_patern)
    {
        // Call on libArachne:
        WallToolPaths wall_tool_paths(skin_part.outline, skin_line_width, concentric_skinfill_patern ? -1 : skin_inset_count, mesh.settings);
        skin_part.inset_paths = wall_tool_paths.getToolPaths();
        skin_part.inner_infill = wall_tool_paths.getInnerContour();
    }
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * generateInfill read mesh.layers[n].parts[*].{insets,skin_parts,boundingBox} and write mesh.layers[n].parts[*].infill_area
 */
void SkinInfillAreaComputation::generateInfill(SliceLayerPart& part, const Polygons& skin)
{
    if (part.insets.size() < wall_line_count)
    {
        return; // the last wall is not present, the part should only get inter perimeter gaps, but no infill.
    }

    part.infill_area = part.inner_area.difference(skin); //Generate infill everywhere where there wasn't any skin.
    part.infill_area.removeSmallAreas(MIN_AREA_SIZE);
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * this function may only read/write the skin and infill from the *current* layer.
 */
void SkinInfillAreaComputation::generateRoofing(SliceLayerPart& part)
{
    const size_t roofing_layer_count = mesh.settings.get<size_t>("roofing_layer_count");

    for(SkinPart& skin_part : part.skin_parts)
    {
        Polygons roofing;
        if(roofing_layer_count > 0)
        {
            Polygons no_air_above = generateNoAirAbove(part);
            skin_part.roofing_fill = skin_part.inner_infill.difference(no_air_above);
            skin_part.inner_infill = skin_part.inner_infill.intersection(no_air_above);
            const bool concentric_skinfill_pattern =
                   mesh.settings.get<EFillMethod>("roofing_pattern") == EFillMethod::CONCENTRIC
                && mesh.settings.get<EFillMethod>("top_bottom_pattern") != EFillMethod::CONCENTRIC;

            // If the pattern is concentric, ONLY use insets.
            // In this case, we still want to generate skinfill for the roofing layers,
            // but only if the roofing pattern is not concentric.
            if(!skin_part.roofing_fill.empty() && layer_nr > 0)
            {

                // Generate skin insets, regenerate the no_air_above, and recalculate the inner and roofing infills, 
                // taking into account the extra skin wall count (only for the roofing layers).
                generateSkinInsets(skin_part, concentric_skinfill_pattern);
                if(!concentric_skinfill_pattern)
                {
                    regenerateRoofingFillAndInnerInfill(part, skin_part);
                }
            }
            // On the contrary, unwanted insets are generated for roofing layers because of the non-concentric top/bottom pattern.
            // In such cases we want to clear the skin insets first and then regenerate the proper roofing fill and inner infill
            // in the concentric roofing_pattern.
            else if(!skin_part.roofing_fill.empty() && skin_part.inner_infill.empty() && layer_nr > 0 && concentric_skinfill_pattern)
            {
                // Clear the skin insets for the roofing layers and regenerate the roofing fill and inner infill without taking into
                // account the Extra Skin Wall Count.
                skin_part.inset_paths.clear();
                regenerateRoofingFillAndInnerInfill(part, skin_part);
            }
        }
    }
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * this function may only read the skin and infill from the *current* layer.
 */
Polygons SkinInfillAreaComputation::generateNoAirAbove(SliceLayerPart& part)
{
    const size_t roofing_layer_count = mesh.settings.get<size_t>("roofing_layer_count");

    Polygons no_air_above = getOutlineOnLayer(part, layer_nr + roofing_layer_count);
    if (!no_small_gaps_heuristic)
    {
        for (int layer_nr_above = layer_nr + 1; layer_nr_above < layer_nr + roofing_layer_count; layer_nr_above++)
        {
            Polygons outlines_above = getOutlineOnLayer(part, layer_nr_above);
            no_air_above = no_air_above.intersection(outlines_above);
        }
    }
    if (layer_nr > 0)
    {
        // if the skin has air below it then cutting it into regions could cause a region
        // to be wholely or partly above air and it may not be printable so restrict
        // the regions that have air above (the visible regions) to not include any area that
        // has air below (fixes https://github.com/Ultimaker/Cura/issues/2656)

        // set air_below to the skin area for the current layer that has air below it
        Polygons air_below = getOutlineOnLayer(part, layer_nr).difference(getOutlineOnLayer(part, layer_nr - 1));

        if (!air_below.empty())
        {
            // add the polygons that have air below to the no air above polygons
            no_air_above = no_air_above.unionPolygons(air_below);
        }
    }
    return no_air_above;
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * this function may only read/write the skin and infill from the *current* layer.
 */
void SkinInfillAreaComputation::regenerateRoofingFillAndInnerInfill(SliceLayerPart& part, SkinPart& skin_part)
{
    Polygons no_air_above = generateNoAirAbove(part);
    skin_part.roofing_fill = skin_part.inner_infill.difference(no_air_above);
    skin_part.inner_infill = skin_part.inner_infill.intersection(no_air_above);
}

void SkinInfillAreaComputation::generateInfillSupport(SliceMeshStorage& mesh)
{
    const coord_t layer_height = mesh.settings.get<coord_t>("layer_height");
    const AngleRadians support_angle = mesh.settings.get<AngleRadians>("infill_support_angle");
    const double tan_angle = tan(support_angle) - 0.01;  //The X/Y component of the support angle. 0.01 to make 90 degrees work too.
    const coord_t max_dist_from_lower_layer = tan_angle * layer_height; //Maximum horizontal distance that can be bridged.

    for (int layer_idx = mesh.layers.size() - 2; layer_idx >= 0; layer_idx--)
    {
        SliceLayer& layer = mesh.layers[layer_idx];
        SliceLayer& layer_above = mesh.layers[layer_idx + 1];

        Polygons inside_above;
        Polygons infill_above;
        for (SliceLayerPart& part_above : layer_above.parts)
        {
            inside_above.add(part_above.infill_area);
            infill_above.add(part_above.getOwnInfillArea());
        }

        for (SliceLayerPart& part : layer.parts)
        {
            const Polygons& infill_area = part.infill_area;
            if (infill_area.empty())
            {
                continue;
            }

            const Polygons unsupported = infill_area.offset(-max_dist_from_lower_layer);
            const Polygons basic_overhang = unsupported.difference(inside_above);
            const Polygons overhang_extented = basic_overhang.offset(max_dist_from_lower_layer + 50); // +50 for easier joining with support from layer above
            const Polygons full_overhang = overhang_extented.difference(inside_above);
            const Polygons infill_support = infill_above.unionPolygons(full_overhang);

            part.infill_area_own = infill_support.intersection(part.getOwnInfillArea());
        }
    }
}

void SkinInfillAreaComputation::generateGradualInfill(SliceMeshStorage& mesh)
{
    // no early-out for this function; it needs to initialize the [infill_area_per_combine_per_density]
    float layer_skip_count = 8; // skip every so many layers as to ignore small gaps in the model making computation more easy
    if (!mesh.settings.get<bool>("skin_no_small_gaps_heuristic"))
    {
        layer_skip_count = 1;
    }
    const coord_t gradual_infill_step_height = mesh.settings.get<coord_t>("gradual_infill_step_height");
    const size_t gradual_infill_step_layer_count = round_divide(gradual_infill_step_height, mesh.settings.get<coord_t>("layer_height")); // The difference in layer count between consecutive density infill areas

    // make gradual_infill_step_height divisible by layer_skip_count
    float n_skip_steps_per_gradual_step = std::max(1.0f, std::ceil(gradual_infill_step_layer_count / layer_skip_count)); // only decrease layer_skip_count to make it a divisor of gradual_infill_step_layer_count
    layer_skip_count = gradual_infill_step_layer_count / n_skip_steps_per_gradual_step;
    const size_t max_infill_steps = mesh.settings.get<size_t>("gradual_infill_steps");

    const LayerIndex min_layer = mesh.settings.get<size_t>("initial_bottom_layers");
    const LayerIndex max_layer = mesh.layers.size() - 1 - mesh.settings.get<size_t>("top_layers");

    for (LayerIndex layer_idx = 0; layer_idx < static_cast<LayerIndex>(mesh.layers.size()); layer_idx++)
    { // loop also over layers which don't contain infill cause of bottom_ and top_layer to initialize their infill_area_per_combine_per_density
        SliceLayer& layer = mesh.layers[layer_idx];

        for (SliceLayerPart& part : layer.parts)
        {
            assert(part.infill_area_per_combine_per_density.size() == 0 && "infill_area_per_combine_per_density is supposed to be uninitialized");

            const Polygons& infill_area = part.getOwnInfillArea();

            if (infill_area.size() == 0 || layer_idx < min_layer || layer_idx > max_layer)
            { // initialize infill_area_per_combine_per_density empty
                part.infill_area_per_combine_per_density.emplace_back(); // create a new infill_area_per_combine
                part.infill_area_per_combine_per_density.back().emplace_back(); // put empty infill area in the newly constructed infill_area_per_combine
                // note: no need to copy part.infill_area, cause it's the empty vector anyway
                continue;
            }
            Polygons less_dense_infill = infill_area; // one step less dense with each infill_step
            for (size_t infill_step = 0; infill_step < max_infill_steps; infill_step++)
            {
                LayerIndex min_layer = layer_idx + infill_step * gradual_infill_step_layer_count + static_cast<size_t>(layer_skip_count);
                LayerIndex max_layer = layer_idx + (infill_step + 1) * gradual_infill_step_layer_count;

                for (float upper_layer_idx = min_layer; upper_layer_idx <= max_layer; upper_layer_idx += layer_skip_count)
                {
                    if (upper_layer_idx >= mesh.layers.size())
                    {
                        less_dense_infill.clear();
                        break;
                    }
                    const SliceLayer& upper_layer = mesh.layers[static_cast<size_t>(upper_layer_idx)];
                    Polygons relevent_upper_polygons;
                    for (const SliceLayerPart& upper_layer_part : upper_layer.parts)
                    {
                        if (!upper_layer_part.boundaryBox.hit(part.boundaryBox))
                        {
                            continue;
                        }
                        relevent_upper_polygons.add(upper_layer_part.getOwnInfillArea());
                    }
                    less_dense_infill = less_dense_infill.intersection(relevent_upper_polygons);
                }
                if (less_dense_infill.size() == 0)
                {
                    break;
                }
                // add new infill_area_per_combine for the current density
                part.infill_area_per_combine_per_density.emplace_back();
                std::vector<Polygons>& infill_area_per_combine_current_density = part.infill_area_per_combine_per_density.back();
                const Polygons more_dense_infill = infill_area.difference(less_dense_infill);
                infill_area_per_combine_current_density.push_back(more_dense_infill);
            }
            part.infill_area_per_combine_per_density.emplace_back();
            std::vector<Polygons>& infill_area_per_combine_current_density = part.infill_area_per_combine_per_density.back();
            infill_area_per_combine_current_density.push_back(infill_area);
            part.infill_area_own = nullptr; // clear infill_area_own, it's not needed any more.
            assert(part.infill_area_per_combine_per_density.size() != 0 && "infill_area_per_combine_per_density is now initialized");
        }
    }
}

void SkinInfillAreaComputation::combineInfillLayers(SliceMeshStorage& mesh)
{
    if (mesh.layers.empty() || mesh.layers.size() - 1 < static_cast<size_t>(mesh.settings.get<size_t>("top_layers")) || mesh.settings.get<coord_t>("infill_line_distance") == 0) //No infill is even generated.
    {
        return;
    }

    const coord_t layer_height = mesh.settings.get<coord_t>("layer_height");
    const size_t amount = std::max(1U, round_divide(mesh.settings.get<coord_t>("infill_sparse_thickness"), std::max(layer_height, coord_t(1)))); //How many infill layers to combine to obtain the requested sparse thickness.
    if(amount <= 1) //If we must combine 1 layer, nothing needs to be combined. Combining 0 layers is invalid.
    {
        return;
    }

    /* We need to round down the layer index we start at to the nearest
    divisible index. Otherwise we get some parts that have infill at divisible
    layers and some at non-divisible layers. Those layers would then miss each
    other. */
    size_t bottom_most_layers = mesh.settings.get<size_t>("initial_bottom_layers");
    LayerIndex min_layer = static_cast<LayerIndex>(bottom_most_layers + amount) - 1;
    min_layer -= min_layer % amount; //Round upwards to the nearest layer divisible by infill_sparse_combine.
    LayerIndex max_layer = static_cast<LayerIndex>(mesh.layers.size()) - 1 - mesh.settings.get<size_t>("top_layers");
    max_layer -= max_layer % amount; //Round downwards to the nearest layer divisible by infill_sparse_combine.
    for(LayerIndex layer_idx = min_layer; layer_idx <= max_layer; layer_idx += amount) //Skip every few layers, but extrude more.
    {
        SliceLayer* layer = &mesh.layers[layer_idx];
        for(size_t combine_count_here = 1; combine_count_here < amount; combine_count_here++)
        {
            if(layer_idx < static_cast<LayerIndex>(combine_count_here))
            {
                break;
            }

            LayerIndex lower_layer_idx = layer_idx - combine_count_here;
            if (lower_layer_idx < min_layer)
            {
                break;
            }
            SliceLayer* lower_layer = &mesh.layers[lower_layer_idx];
            for (SliceLayerPart& part : layer->parts)
            {
                for (unsigned int density_idx = 0; density_idx < part.infill_area_per_combine_per_density.size(); density_idx++)
                { // go over each density of gradual infill (these density areas overlap!)
                    std::vector<Polygons>& infill_area_per_combine = part.infill_area_per_combine_per_density[density_idx];
                    Polygons result;
                    for (SliceLayerPart& lower_layer_part : lower_layer->parts)
                    {
                        if (part.boundaryBox.hit(lower_layer_part.boundaryBox))
                        {

                            Polygons intersection = infill_area_per_combine[combine_count_here - 1].intersection(lower_layer_part.infill_area).offset(-200).offset(200);
                            result.add(intersection); // add area to be thickened
                            infill_area_per_combine[combine_count_here - 1] = infill_area_per_combine[combine_count_here - 1].difference(intersection); // remove thickened area from less thick layer here
                            unsigned int max_lower_density_idx = density_idx;
                            // Generally: remove only from *same density* areas on layer below
                            // If there are no same density areas, then it's ok to print them anyway
                            // Don't remove other density areas
                            if (density_idx == part.infill_area_per_combine_per_density.size() - 1)
                            {
                                // For the most dense areas on a given layer the density of that area is doubled.
                                // This means that - if the lower layer has more densities -
                                // all those lower density lines are included in the most dense of this layer.
                                // We therefore compare the most dense are on this layer with all densities
                                // of the lower layer with the same or higher density index
                                max_lower_density_idx = lower_layer_part.infill_area_per_combine_per_density.size() - 1;
                            }
                            for (size_t lower_density_idx = density_idx; lower_density_idx <= max_lower_density_idx && lower_density_idx < lower_layer_part.infill_area_per_combine_per_density.size(); lower_density_idx++)
                            {
                                std::vector<Polygons>& lower_infill_area_per_combine = lower_layer_part.infill_area_per_combine_per_density[lower_density_idx];
                                lower_infill_area_per_combine[0] = lower_infill_area_per_combine[0].difference(intersection); // remove thickened area from lower (single thickness) layer
                            }
                        }
                    }

                    infill_area_per_combine.push_back(result);
                }
            }
        }
    }
}


}//namespace cura
