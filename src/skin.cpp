// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "skin.h"

#include <cmath> // std::ceil

#include <spdlog/spdlog.h>

#include "Application.h" //To get settings.
#include "ExtruderTrain.h"
#include "Slice.h"
#include "WallToolPaths.h"
#include "infill.h"
#include "settings/EnumSettings.h" //For EFillMethod.
#include "settings/types/Angle.h" //For the infill support angle.
#include "settings/types/Ratio.h"
#include "sliceDataStorage.h"
#include "utils/Simplify.h"
#include "utils/math.h"
#include "utils/polygonUtils.h"

#define MIN_AREA_SIZE (0.4 * 0.4)

namespace cura
{

coord_t SkinInfillAreaComputation::getSkinLineWidth(const SliceMeshStorage& mesh, const LayerIndex& layer_nr)
{
    coord_t skin_line_width = mesh.settings.get<coord_t>("skin_line_width");
    if (layer_nr == 0)
    {
        const ExtruderTrain& train_skin = mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr");
        skin_line_width *= train_skin.settings_.get<Ratio>("initial_layer_line_width_factor");
    }
    return skin_line_width;
}

SkinInfillAreaComputation::SkinInfillAreaComputation(const LayerIndex& layer_nr, SliceMeshStorage& mesh, bool process_infill)
    : layer_nr_(layer_nr)
    , mesh_(mesh)
    , bottom_layer_count_(mesh.settings.get<size_t>("bottom_layers"))
    , initial_bottom_layer_count_(mesh.settings.get<size_t>("initial_bottom_layers"))
    , top_layer_count_(mesh.settings.get<size_t>("top_layers"))
    , skin_line_width_(getSkinLineWidth(mesh, layer_nr))
    , no_small_gaps_heuristic_(mesh.settings.get<bool>("skin_no_small_gaps_heuristic"))
    , process_infill_(process_infill)
    , top_skin_preshrink_(mesh.settings.get<coord_t>("top_skin_preshrink"))
    , bottom_skin_preshrink_(mesh.settings.get<coord_t>("bottom_skin_preshrink"))
    , top_skin_expand_distance_(mesh.settings.get<coord_t>("top_skin_expand_distance"))
    , bottom_skin_expand_distance_(mesh.settings.get<coord_t>("bottom_skin_expand_distance"))
{
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * this function may only read/write the skin and infill from the *current* layer.
 */
Shape SkinInfillAreaComputation::getOutlineOnLayer(const SliceLayerPart& part_here, const LayerIndex layer2_nr)
{
    Shape result;
    if (layer2_nr >= static_cast<int>(mesh_.layers.size()))
    {
        return result;
    }
    const SliceLayer& layer2 = mesh_.layers[layer2_nr];
    for (const SliceLayerPart& part2 : layer2.parts)
    {
        if (part_here.boundaryBox.hit(part2.boundaryBox))
        {
            result.push_back(part2.outline);
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

    SliceLayer* layer = &mesh_.layers[layer_nr_];

    for (SliceLayerPart& part : layer->parts)
    {
        generateSkinRoofingFlooringFill(part);

        generateTopAndBottomMostSurfaces(part);
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
    SliceLayer& layer = mesh_.layers[layer_nr_];

    if (! process_infill_ && bottom_layer_count_ == 0 && top_layer_count_ == 0)
    {
        return;
    }

    for (SliceLayerPart& part : layer.parts)
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
    // Make a copy of the outline which we later intersect and union with the resized skins to ensure the resized skin isn't too large or removed completely.
    Shape top_skin;
    if (top_layer_count_ > 0)
    {
        top_skin = Shape(part.inner_area);
    }
    Shape bottom_skin;
    if (bottom_layer_count_ > 0 || layer_nr_ < LayerIndex(initial_bottom_layer_count_))
    {
        bottom_skin = Shape(part.inner_area);
    }

    calculateBottomSkin(part, bottom_skin);
    calculateTopSkin(part, top_skin);

    applySkinExpansion(part.inner_area, top_skin, bottom_skin);

    // Now combine the resized top skin and bottom skin.
    Shape skin = top_skin.unionPolygons(bottom_skin);

    skin.removeSmallAreas(MIN_AREA_SIZE);
    // Create infill area irrespective if the infill is to be generated or not(would be used for bridging).
    part.infill_area = part.inner_area.difference(skin);
    if (process_infill_)
    { // process infill when infill density > 0
        // or when other infill meshes want to modify this infill
        generateInfill(part);
    }

    for (const SingleShape& skin_area_part : skin.splitIntoParts())
    {
        if (skin_area_part.empty())
        {
            continue;
        }
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
void SkinInfillAreaComputation::calculateBottomSkin(const SliceLayerPart& part, Shape& downskin)
{
    if (bottom_layer_count_ == 0 && initial_bottom_layer_count_ == 0)
    {
        return; // downskin remains empty
    }
    if (layer_nr_ < LayerIndex(initial_bottom_layer_count_))
    {
        return; // don't subtract anything form the downskin
    }
    LayerIndex bottom_check_start_layer_idx{ std::max(LayerIndex{ 0 }, LayerIndex{ layer_nr_ - bottom_layer_count_ }) };
    Shape not_air = getOutlineOnLayer(part, bottom_check_start_layer_idx);
    if (! no_small_gaps_heuristic_)
    {
        for (int downskin_layer_nr = bottom_check_start_layer_idx + 1; downskin_layer_nr < layer_nr_; downskin_layer_nr++)
        {
            not_air = not_air.intersection(getOutlineOnLayer(part, downskin_layer_nr));
        }
    }
    const double min_infill_area = mesh_.settings.get<double>("min_infill_area");
    if (min_infill_area > 0.0)
    {
        not_air.removeSmallAreas(min_infill_area);
    }
    downskin = downskin.difference(not_air); // skin overlaps with the walls
}

void SkinInfillAreaComputation::calculateTopSkin(const SliceLayerPart& part, Shape& upskin)
{
    if (layer_nr_ > LayerIndex(mesh_.layers.size()) - top_layer_count_ || top_layer_count_ <= 0)
    {
        // If we're in the very top layers (less than top_layer_count from the top of the mesh) everything will be top skin anyway, so no need to generate infill. Just take the
        // original inner contour. If top_layer_count is 0, no need to calculate anything either.
        return;
    }

    Shape not_air = getOutlineOnLayer(part, layer_nr_ + top_layer_count_);
    if (! no_small_gaps_heuristic_)
    {
        for (int upskin_layer_nr = layer_nr_ + 1; upskin_layer_nr < layer_nr_ + top_layer_count_; upskin_layer_nr++)
        {
            not_air = not_air.intersection(getOutlineOnLayer(part, upskin_layer_nr));
        }
    }

    const double min_infill_area = mesh_.settings.get<double>("min_infill_area");
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
void SkinInfillAreaComputation::applySkinExpansion(const Shape& original_outline, Shape& upskin, Shape& downskin)
{
    const coord_t min_width = mesh_.settings.get<coord_t>("min_skin_width_for_expansion") / 2;

    // Expand some areas of the skin for Skin Expand Distance.
    if (min_width > 0)
    {
        // This performs an opening operation by first insetting by the minimum width, then offsetting with the same width.
        // The expansion is only applied to that opened shape.
        if (bottom_skin_expand_distance_ != 0)
        {
            const Shape expanded = downskin.offset(-min_width).offset(min_width + bottom_skin_expand_distance_);
            // And then re-joined with the original part that was not offset, to retain parts smaller than min_width.
            downskin = downskin.unionPolygons(expanded);
        }
        if (top_skin_expand_distance_ != 0)
        {
            const Shape expanded = upskin.offset(-min_width).offset(min_width + top_skin_expand_distance_);
            upskin = upskin.unionPolygons(expanded);
        }
    }
    else // No need to pay attention to minimum width. Just expand.
    {
        if (bottom_skin_expand_distance_ != 0)
        {
            downskin = downskin.offset(bottom_skin_expand_distance_);
        }
        if (top_skin_expand_distance_ != 0)
        {
            upskin = upskin.offset(top_skin_expand_distance_);
        }
    }

    bool should_bottom_be_clipped = bottom_skin_expand_distance_ > 0;
    bool should_top_be_clipped = top_skin_expand_distance_ > 0;

    // Set miter limit for morphological open in order to fill in
    // "narrow point" features of polygons due to offsetting artifacts.
    //
    //                    --  <- cap due to miter limit
    //         offset    /  \    offset
    //    /\  =======>  /    \  =========>   __  <- same cap from previous
    //   /  \ outwards /      \  inwards    /  \    offset-miter artifact
    //
    // By setting the miter limit to a very high value the outward
    // offsetted polygon is longer "capped". Usually a downside of
    // setting the miter limit this high is that for arbitrary small angles
    // the offsetted polygon becomes arbitrary large. This is not an issue
    // as the final polygon is limited (intersected) with the original polygon.
    constexpr double MITER_LIMIT = 10000000.0;
    // Remove thin pieces of support for Skin Removal Width.
    if (bottom_skin_preshrink_ > 0 || (min_width == 0 && bottom_skin_expand_distance_ != 0))
    {
        downskin = downskin.offset(-bottom_skin_preshrink_ / 2, ClipperLib::jtMiter, MITER_LIMIT)
                       .offset(bottom_skin_preshrink_ / 2, ClipperLib::jtMiter, MITER_LIMIT)
                       .intersection(downskin);
        should_bottom_be_clipped = true; // Rounding errors can lead to propagation of errors. This could mean that skin goes beyond the original outline
    }
    if (top_skin_preshrink_ > 0 || (min_width == 0 && top_skin_expand_distance_ != 0))
    {
        upskin = upskin.offset(-top_skin_preshrink_ / 2, ClipperLib::jtMiter, MITER_LIMIT).offset(top_skin_preshrink_ / 2, ClipperLib::jtMiter, MITER_LIMIT);
        should_top_be_clipped = true; // Rounding errors can lead to propagation of errors. This could mean that skin goes beyond the original outline
    }

    if (should_bottom_be_clipped)
    {
        downskin = downskin.intersection(original_outline);
    }
    if (should_top_be_clipped)
    {
        upskin = upskin.intersection(original_outline);
    }
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * generateInfill read mesh.layers[n].parts[*].{insets,skin_parts,boundingBox} and write mesh.layers[n].parts[*].infill_area
 */
void SkinInfillAreaComputation::generateInfill(SliceLayerPart& part)
{
    // Generate infill everywhere where there wasn't any skin.
    part.infill_area.removeSmallAreas(MIN_AREA_SIZE);
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * this function may only read/write the skin and infill from the *current* layer.
 */
void SkinInfillAreaComputation::generateSkinRoofingFlooringFill(SliceLayerPart& part)
{
    for (SkinPart& skin_part : part.skin_parts)
    {
        const size_t roofing_layer_count = std::min(mesh_.settings.get<size_t>("roofing_layer_count"), mesh_.settings.get<size_t>("top_layers"));
        const size_t flooring_layer_count = std::min(mesh_.settings.get<size_t>("flooring_layer_count"), mesh_.settings.get<size_t>("bottom_layers"));
        const coord_t skin_overlap = mesh_.settings.get<coord_t>("skin_overlap_mm");

        const Shape filled_area_above = generateFilledAreaAbove(part, roofing_layer_count);
        const std::optional<Shape> filled_area_below = generateFilledAreaBelow(part, flooring_layer_count);

        // An area that would have nothing below nor above is considered a roof
        skin_part.roofing_fill = skin_part.outline.difference(filled_area_above);
        if (filled_area_below.has_value())
        {
            skin_part.flooring_fill = skin_part.outline.intersection(filled_area_above).difference(*filled_area_below);
            skin_part.skin_fill = skin_part.outline.intersection(filled_area_above).intersection(*filled_area_below);
        }
        else
        {
            // Mesh part is just above build plate, so it is completely supported
            // skin_part.flooring_fill = Shape();
            skin_part.skin_fill = skin_part.outline.intersection(filled_area_above);
        }

        // We remove offsets areas from roofing and flooring anywhere they overlap with skin_fill.
        // Otherwise, adjacent skin_fill and roofing/flooring would have doubled offset areas. Since they both offset into each other.
        skin_part.skin_fill = skin_part.skin_fill.offset(skin_overlap).difference(skin_part.roofing_fill).difference(skin_part.flooring_fill);
        skin_part.roofing_fill = skin_part.roofing_fill.offset(skin_overlap);
        skin_part.flooring_fill = skin_part.flooring_fill.offset(skin_overlap).difference(skin_part.roofing_fill);
    }
}

void SkinInfillAreaComputation::generateTopAndBottomMostSurfaces(SliceLayerPart& part)
{
    const Shape outline_above = getOutlineOnLayer(part, layer_nr_ + 1);
    part.top_most_surface = part.outline.difference(outline_above);

    if (layer_nr_ > 0)
    {
        const Shape outline_below = getOutlineOnLayer(part, layer_nr_ + 1);
        part.bottom_most_surface = part.outline.difference(outline_below);
    }
    else
    {
        part.bottom_most_surface = part.outline;
    }
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * this function may only read the skin and infill from the *current* layer.
 */
Shape SkinInfillAreaComputation::generateFilledAreaAbove(SliceLayerPart& part, size_t roofing_layer_count)
{
    Shape filled_area_above = getOutlineOnLayer(part, layer_nr_ + roofing_layer_count);
    if (! no_small_gaps_heuristic_)
    {
        for (int layer_nr_above = layer_nr_ + 1; layer_nr_above < layer_nr_ + roofing_layer_count; layer_nr_above++)
        {
            Shape outlines_above = getOutlineOnLayer(part, layer_nr_above);
            filled_area_above = filled_area_above.intersection(outlines_above);
        }
    }
    if (layer_nr_ > 0)
    {
        // if the skin has air below it then cutting it into regions could cause a region
        // to be wholely or partly above air and it may not be printable so restrict
        // the regions that have air above (the visible regions) to not include any area that
        // has air below (fixes https://github.com/Ultimaker/Cura/issues/2656)

        // set air_below to the skin area for the current layer that has air below it
        Shape air_below = getOutlineOnLayer(part, layer_nr_).difference(getOutlineOnLayer(part, layer_nr_ - 1));

        if (! air_below.empty())
        {
            // add the polygons that have air below to the no air above polygons
            filled_area_above = filled_area_above.unionPolygons(air_below);
        }
    }
    return filled_area_above;
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * this function may only read the skin and infill from the *current* layer.
 */
std::optional<Shape> SkinInfillAreaComputation::generateFilledAreaBelow(SliceLayerPart& part, size_t flooring_layer_count)
{
    if (layer_nr_ < flooring_layer_count)
    {
        return std::nullopt;
    }

    const int lowest_flooring_layer = layer_nr_ - flooring_layer_count;
    Shape filled_area_below = getOutlineOnLayer(part, lowest_flooring_layer);

    if (! no_small_gaps_heuristic_)
    {
        const int next_lowest_flooring_layer = lowest_flooring_layer + 1;
        for (int layer_nr_below = next_lowest_flooring_layer; layer_nr_below < layer_nr_; layer_nr_below++)
        {
            Shape outlines_below = getOutlineOnLayer(part, layer_nr_below);
            filled_area_below = filled_area_below.intersection(outlines_below);
        }
    }
    return filled_area_below;
}

void SkinInfillAreaComputation::generateInfillSupport(SliceMeshStorage& mesh)
{
    const coord_t layer_height = mesh.settings.get<coord_t>("layer_height");
    const AngleRadians support_angle = mesh.settings.get<AngleRadians>("infill_support_angle");
    const double tan_angle = tan(support_angle) - 0.01; // The X/Y component of the support angle. 0.01 to make 90 degrees work too.
    const coord_t max_dist_from_lower_layer = tan_angle * layer_height; // Maximum horizontal distance that can be bridged.

    for (int layer_idx = mesh.layers.size() - 2; layer_idx >= 0; layer_idx--)
    {
        SliceLayer& layer = mesh.layers[layer_idx];
        SliceLayer& layer_above = mesh.layers[layer_idx + 1];

        Shape inside_above;
        Shape infill_above;
        for (SliceLayerPart& part_above : layer_above.parts)
        {
            inside_above.push_back(part_above.infill_area);
            infill_above.push_back(part_above.getOwnInfillArea());
        }

        for (SliceLayerPart& part : layer.parts)
        {
            const Shape& infill_area = part.infill_area;
            if (infill_area.empty())
            {
                continue;
            }

            const Shape unsupported = infill_area.offset(-max_dist_from_lower_layer);
            const Shape basic_overhang = unsupported.difference(inside_above);
            const Shape overhang_extented = basic_overhang.offset(max_dist_from_lower_layer + 50); // +50 for easier joining with support from layer above
            const Shape full_overhang = overhang_extented.difference(inside_above);
            const Shape infill_support = infill_above.unionPolygons(full_overhang);

            part.infill_area_own = infill_support.intersection(part.getOwnInfillArea());
        }
    }
}

void SkinInfillAreaComputation::generateGradualInfill(SliceMeshStorage& mesh)
{
    // no early-out for this function; it needs to initialize the [infill_area_per_combine_per_density]
    double layer_skip_count = 8; // skip every so many layers as to ignore small gaps in the model making computation more easy
    if (! mesh.settings.get<bool>("skin_no_small_gaps_heuristic"))
    {
        layer_skip_count = 1;
    }
    const coord_t gradual_infill_step_height = mesh.settings.get<coord_t>("gradual_infill_step_height");
    const size_t gradual_infill_step_layer_count
        = round_divide(gradual_infill_step_height, mesh.settings.get<coord_t>("layer_height")); // The difference in layer count between consecutive density infill areas

    // make gradual_infill_step_height divisible by layer_skip_count
    double n_skip_steps_per_gradual_step
        = std::max(1.0, std::ceil(gradual_infill_step_layer_count / layer_skip_count)); // only decrease layer_skip_count to make it a divisor of gradual_infill_step_layer_count
    layer_skip_count = gradual_infill_step_layer_count / n_skip_steps_per_gradual_step;
    const size_t max_infill_steps = mesh.settings.get<size_t>("gradual_infill_steps");

    const LayerIndex mesh_min_layer = mesh.settings.get<size_t>("initial_bottom_layers");
    const LayerIndex mesh_max_layer = mesh.layers.size() - 1 - mesh.settings.get<size_t>("top_layers");

    const Simplify simplifier(mesh.settings.get<ExtruderTrain&>("infill_extruder_nr").settings_);

    const auto infill_wall_count = mesh.settings.get<size_t>("infill_wall_line_count");
    const auto infill_wall_width = mesh.settings.get<coord_t>("infill_line_width");
    const auto is_connected = mesh.settings.get<bool>("zig_zaggify_infill") || mesh.settings.get<EFillMethod>("infill_pattern") == EFillMethod::ZIG_ZAG;
    for (LayerIndex layer_idx = 0; layer_idx < static_cast<LayerIndex>(mesh.layers.size()); layer_idx++)
    { // loop also over layers which don't contain infill cause of bottom_ and top_layer to initialize their infill_area_per_combine_per_density
        SliceLayer& layer = mesh.layers[layer_idx];

        for (SliceLayerPart& part : layer.parts)
        {
            assert((part.infill_area_per_combine_per_density.empty() && "infill_area_per_combine_per_density is supposed to be uninitialized"));

            const Shape& infill_area = Infill::generateWallToolPaths(
                part.infill_wall_toolpaths,
                part.getOwnInfillArea(),
                infill_wall_count,
                infill_wall_width,
                mesh.settings,
                layer_idx,
                SectionType::SKIN);

            if (infill_area.empty() || layer_idx < mesh_min_layer || layer_idx > mesh_max_layer)
            { // initialize infill_area_per_combine_per_density empty
                part.infill_area_per_combine_per_density.emplace_back(); // create a new infill_area_per_combine
                part.infill_area_per_combine_per_density.back().emplace_back(); // put empty infill area in the newly constructed infill_area_per_combine
                // note: no need to copy part.infill_area, cause it's the empty vector anyway
                continue;
            }
            Shape less_dense_infill = infill_area; // one step less dense with each infill_step
            Shape sum_more_dense; // NOTE: Only used for zig-zag or connected fills.
            for (size_t infill_step = 0; infill_step < max_infill_steps; infill_step++)
            {
                LayerIndex min_layer = layer_idx + infill_step * gradual_infill_step_layer_count + static_cast<size_t>(layer_skip_count);
                LayerIndex max_layer = layer_idx + (infill_step + 1) * gradual_infill_step_layer_count;

                for (double upper_layer_idx = min_layer; upper_layer_idx <= max_layer; upper_layer_idx += layer_skip_count)
                {
                    if (upper_layer_idx >= mesh.layers.size())
                    {
                        less_dense_infill.clear();
                        break;
                    }
                    const SliceLayer& upper_layer = mesh.layers[static_cast<size_t>(upper_layer_idx)];
                    Shape relevent_upper_polygons;
                    for (const SliceLayerPart& upper_layer_part : upper_layer.parts)
                    {
                        if (! upper_layer_part.boundaryBox.hit(part.boundaryBox))
                        {
                            continue;
                        }
                        relevent_upper_polygons.push_back(upper_layer_part.getOwnInfillArea());
                    }
                    less_dense_infill = less_dense_infill.intersection(relevent_upper_polygons);
                }
                if (less_dense_infill.empty())
                {
                    break;
                }
                // add new infill_area_per_combine for the current density
                part.infill_area_per_combine_per_density.emplace_back();
                std::vector<Shape>& infill_area_per_combine_current_density = part.infill_area_per_combine_per_density.back();
                const Shape more_dense_infill = infill_area.difference(less_dense_infill);
                infill_area_per_combine_current_density.push_back(simplifier.polygon(more_dense_infill.difference(sum_more_dense)));
                if (is_connected)
                {
                    sum_more_dense = sum_more_dense.unionPolygons(more_dense_infill);
                }
            }
            part.infill_area_per_combine_per_density.emplace_back();
            std::vector<Shape>& infill_area_per_combine_current_density = part.infill_area_per_combine_per_density.back();
            infill_area_per_combine_current_density.push_back(simplifier.polygon(infill_area.difference(sum_more_dense)));
            part.infill_area_own = std::nullopt; // clear infill_area_own, it's not needed any more.
            assert(! part.infill_area_per_combine_per_density.empty() && "infill_area_per_combine_per_density is now initialized");
        }
    }
}

void SkinInfillAreaComputation::combineInfillLayers(SliceMeshStorage& mesh)
{
    if (mesh.layers.empty() || mesh.layers.size() - 1 < mesh.settings.get<size_t>("top_layers")
        || mesh.settings.get<coord_t>("infill_line_distance") == 0) // No infill is even generated.
    {
        return;
    }

    const coord_t layer_height = mesh.settings.get<coord_t>("layer_height");
    const size_t amount = std::max(
        uint64_t(1),
        round_divide(
            mesh.settings.get<coord_t>("infill_sparse_thickness"),
            std::max(layer_height, coord_t(1)))); // How many infill layers to combine to obtain the requested sparse thickness.
    if (amount <= 1) // If we must combine 1 layer, nothing needs to be combined. Combining 0 layers is invalid.
    {
        return;
    }

    /* We need to round down the layer index we start at to the nearest
    divisible index. Otherwise we get some parts that have infill at divisible
    layers and some at non-divisible layers. Those layers would then miss each
    other. */
    size_t bottom_most_layers = mesh.settings.get<size_t>("initial_bottom_layers");
    LayerIndex min_layer = static_cast<LayerIndex>(bottom_most_layers + amount) - 1;
    min_layer -= min_layer % amount; // Round upwards to the nearest layer divisible by infill_sparse_combine.
    LayerIndex max_layer = static_cast<LayerIndex>(mesh.layers.size()) - 1 - mesh.settings.get<size_t>("top_layers");
    max_layer -= max_layer % amount; // Round downwards to the nearest layer divisible by infill_sparse_combine.
    for (LayerIndex layer_idx = min_layer; layer_idx <= max_layer; layer_idx += amount) // Skip every few layers, but extrude more.
    {
        SliceLayer* layer = &mesh.layers[layer_idx];
        for (size_t combine_count_here = 1; combine_count_here < amount; combine_count_here++)
        {
            if (layer_idx < static_cast<LayerIndex>(combine_count_here))
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
                    std::vector<Shape>& infill_area_per_combine = part.infill_area_per_combine_per_density[density_idx];
                    Shape result;
                    for (SliceLayerPart& lower_layer_part : lower_layer->parts)
                    {
                        if (part.boundaryBox.hit(lower_layer_part.boundaryBox))
                        {
                            Shape intersection = infill_area_per_combine[combine_count_here - 1].intersection(lower_layer_part.infill_area).offset(-200).offset(200);
                            result.push_back(intersection); // add area to be thickened
                            infill_area_per_combine[combine_count_here - 1]
                                = infill_area_per_combine[combine_count_here - 1].difference(intersection); // remove thickened area from less thick layer here
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
                            for (size_t lower_density_idx = density_idx;
                                 lower_density_idx <= max_lower_density_idx && lower_density_idx < lower_layer_part.infill_area_per_combine_per_density.size();
                                 lower_density_idx++)
                            {
                                std::vector<Shape>& lower_infill_area_per_combine = lower_layer_part.infill_area_per_combine_per_density[lower_density_idx];
                                lower_infill_area_per_combine[0]
                                    = lower_infill_area_per_combine[0].difference(intersection); // remove thickened area from lower (single thickness) layer
                            }
                        }
                    }

                    infill_area_per_combine.push_back(result);
                }
            }
        }
    }
}

} // namespace cura
