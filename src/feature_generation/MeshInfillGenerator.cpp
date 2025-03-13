// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "feature_generation/MeshInfillGenerator.h"

#include <ExtruderTrain.h>
#include <infill.h>
#include <infill/LightningLayer.h>
#include <settings/EnumSettings.h>
#include <settings/MeshPathConfigs.h>
#include <settings/PathConfigStorage.h>
#include <sliceDataStorage.h>

#include "print_operation/ContinuousExtruderMoveSequence.h"
#include "print_operation/ExtruderPlan.h"
#include "print_operation/InfillFeatureExtrusion.h"
#include "print_operation/LayerPlan.h"
#include "utils/math.h"


namespace cura
{

MeshInfillGenerator::MeshInfillGenerator(const std::shared_ptr<SliceMeshStorage>& mesh)
    : MeshFeatureGenerator(mesh)
    , infill_line_distance_(mesh->settings.get<coord_t>("infill_line_distance"))
{
}

void MeshInfillGenerator::preCalculateData()
{
    MeshFeatureGenerator::preCalculateData();

    if (getMesh()->settings.get<EFillMethod>("infill_pattern") == EFillMethod::LIGHTNING)
    {
        lightning_generator_ = std::make_shared<LightningGenerator>(getMesh());
    }
}

bool MeshInfillGenerator::isActive() const
{
    if (! MeshFeatureGenerator::isActive())
    {
        return false;
    }

    if (infill_line_distance_ <= 0)
    {
        return false;
    }

    return true;
}

void MeshInfillGenerator::generateFeatures(
    const SliceDataStorage& /*storage*/,
    const LayerPlanPtr& layer_plan,
    const std::vector<ExtruderPlanPtr>& extruder_plans,
    const SliceLayerPart& part) const
{
    const Settings& settings = getMesh()->settings;
    const ExtruderNumber infill_extruder_nr = settings.get<ExtruderTrain&>("infill_extruder_nr").extruder_nr_;
    const ExtruderPlanPtr extruder_plan_infill = ExtruderPlan::find(extruder_plans, infill_extruder_nr);
    assert(extruder_plan_infill && "Unable to find extruder plan for infill");

    const auto max_resolution = settings.get<coord_t>("meshfix_maximum_resolution");
    const auto max_deviation = settings.get<coord_t>("meshfix_maximum_deviation");

    AngleDegrees infill_angle = 45; // Original default. This will get updated to an element from mesh->infill_angles.
    if (! getMesh()->infill_angles.empty())
    {
        const size_t combined_infill_layers
            = std::max(uint64_t(1), round_divide(settings.get<coord_t>("infill_sparse_thickness"), std::max(settings.get<coord_t>("layer_height"), coord_t(1))));
        infill_angle = getMesh()->infill_angles.at((layer_plan->getLayerIndex() / combined_infill_layers) % getMesh()->infill_angles.size());
    }

    const Point3LL mesh_middle = getMesh()->bounding_box.getMiddle();
    const MeshPathConfigs& mesh_configs = layer_plan->getConfigsStorage()->mesh_configs.at(getMesh());

    const Point2LL infill_origin(mesh_middle.x_ + settings.get<coord_t>("infill_offset_x"), mesh_middle.y_ + settings.get<coord_t>("infill_offset_y"));

    const auto infill_pattern = settings.get<EFillMethod>("infill_pattern");
    const auto zig_zaggify_infill = settings.get<bool>("zig_zaggify_infill") || infill_pattern == EFillMethod::ZIG_ZAG;
    const auto connect_polygons = settings.get<bool>("connect_infill_polygons");
    const auto infill_multiplier = settings.get<size_t>("infill_multiplier");
    const auto infill_overlap = settings.get<coord_t>("infill_overlap_mm");
    const auto pocket_size = settings.get<coord_t>("cross_infill_pocket_size");

    constexpr bool skip_stitching = false;
    constexpr bool connected_zigzags = false;
    constexpr bool skip_some_zags = false;
    constexpr int zag_skip_count = 0;
    constexpr coord_t small_area_width = 0;

    const size_t last_idx = part.infill_area_per_combine_per_density.size() - 1;

    std::shared_ptr<LightningLayer> lightning_layer = nullptr;
    if (lightning_generator_)
    {
        lightning_layer = std::make_shared<LightningLayer>(lightning_generator_->getTreesForLayer(layer_plan->getLayerIndex()));
    }

    for (unsigned int combine_idx = 0; combine_idx < part.infill_area_per_combine_per_density[0].size(); combine_idx++)
    {
        Infill::GeneratedPatterns patterns;

        const GCodePathConfig& infill_config = mesh_configs.infill_config[combine_idx];
        const coord_t infill_line_width = infill_config.getLineWidth();

        const auto generate_infill = [&](const Shape& in_outline,
                                         const coord_t infill_line_distance,
                                         const coord_t overlap,
                                         const coord_t infill_shift,
                                         const coord_t wall_count,
                                         const size_t density_idx,
                                         const bool use_endpieces,
                                         Infill::GeneratedPatterns& patterns)
        {
            const bool fill_gaps = density_idx == 0; // Only fill gaps for the lowest density.

            Infill infill_comp(
                infill_pattern,
                zig_zaggify_infill,
                connect_polygons,
                in_outline,
                infill_line_width,
                infill_line_distance,
                overlap,
                infill_multiplier,
                infill_angle,
                layer_plan->getZ(),
                infill_shift,
                max_resolution,
                max_deviation,
                wall_count,
                small_area_width,
                infill_origin,
                skip_stitching,
                fill_gaps,
                connected_zigzags,
                use_endpieces,
                skip_some_zags,
                zag_skip_count,
                pocket_size);
            infill_comp.generate(patterns, settings, layer_plan->getLayerIndex(), SectionType::INFILL, getMesh()->cross_fill_provider, lightning_layer, getMesh().get());
        };

        if (combine_idx == 0)
        {
            processMultiLayerInfill(part, settings, layer_plan, last_idx, infill_pattern, infill_line_width, infill_overlap, zig_zaggify_infill, generate_infill, patterns);
        }
        else
        {
            processSingleLayerInfill(part, last_idx, infill_pattern, combine_idx, infill_overlap, generate_infill);
        }

        auto feature_extrusion = InfillFeatureExtrusion::makeFrom(patterns, PrintFeatureType::Infill, infill_line_width, getMesh(), infill_angle, infill_config.getSpeed());
        extruder_plan_infill->appendFeatureExtrusion(feature_extrusion);
    }
}

void MeshInfillGenerator::processMultiLayerInfill(
    const SliceLayerPart& part,
    const Settings& settings,
    const LayerPlanPtr& layer_plan,
    const size_t last_idx,
    const EFillMethod infill_pattern,
    const coord_t infill_line_width,
    const coord_t infill_overlap,
    const bool zig_zaggify_infill,
    const auto generate_infill,
    Infill::GeneratedPatterns& patterns) const
{
    const auto wall_line_count = settings.get<size_t>("infill_wall_line_count");

    Shape sparse_in_outline = part.infill_area_per_combine_per_density[last_idx][0];

    // if infill walls are required below the boundaries of skin regions above, partition the infill along the
    // boundary edge
    Shape infill_below_skin;
    Shape infill_not_below_skin;
    const bool hasSkinEdgeSupport = partitionInfillBySkinAbove(infill_below_skin, infill_not_below_skin, layer_plan, part, infill_line_width);

    const bool use_endpieces = part.infill_area_per_combine_per_density.size() == 1; // Only use endpieces when not using gradual infill, since they will then overlap.

    auto get_cut_offset = [](const bool zig_zaggify, const coord_t line_width, const size_t line_count)
    {
        if (zig_zaggify)
        {
            return -line_width / 2 - static_cast<coord_t>(line_count) * line_width - 5;
        }
        return -static_cast<coord_t>(line_count) * line_width;
    };

    for (size_t density_idx = last_idx; static_cast<int>(density_idx) >= 0; density_idx--)
    {
        // Only process dense areas when they're initialized
        if (part.infill_area_per_combine_per_density[density_idx][0].empty())
        {
            continue;
        }

        OpenLinesSet infill_lines_here;
        Shape infill_polygons_here;

        // the highest density infill combines with the next to create a grid with density_factor 1
        int infill_line_distance_here = infill_line_distance_ << (density_idx + 1);
        int infill_shift = infill_line_distance_here / 2;

        /* infill shift explanation: [>]=shift ["]=line_dist

         :       |       :       |       :       |       :       |         > furthest from top
         :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |     > further from top
         : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | |   > near top
         >>"""""
         :       |       :       |       :       |       :       |         > furthest from top
         :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |     > further from top
         : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | |   > near top
         >>>>"""""""""
         :       |       :       |       :       |       :       |         > furthest from top
         :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |     > further from top
         : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | |   > near top
         >>>>>>>>"""""""""""""""""
         */

        // All of that doesn't hold for the Cross patterns; they should just always be multiplied by 2.
        if (density_idx == part.infill_area_per_combine_per_density.size() - 1 || infill_pattern == EFillMethod::CROSS || infill_pattern == EFillMethod::CROSS_3D)
        {
            /* the least dense infill should fill up all remaining gaps
             :       |       :       |       :       |       :       |       :  > furthest from top
             :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |   :  > further from top
             : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | | :  > near top
               .   .     .       .           .               .       .       .
               :   :     :       :           :               :       :       :
               `"""'     `"""""""'           `"""""""""""""""'       `"""""""'
                                                                         ^   new line distance for lowest density infill
                                                   ^ infill_line_distance_here for lowest density infill up till here
                             ^ middle density line dist
                 ^   highest density line dist*/

            // All of that doesn't hold for the Cross patterns; they should just always be multiplied by 2 for every density index.
            infill_line_distance_here /= 2;
        }

        Shape in_outline = part.infill_area_per_combine_per_density[density_idx][0];
        const bool fill_gaps = density_idx == 0; // Only fill gaps in the lowest infill density pattern.
        if (hasSkinEdgeSupport)
        {
            // infill region with skin above has to have at least one infill wall line
            const size_t min_skin_below_wall_count = wall_line_count > 0 ? wall_line_count : 1;
            const size_t skin_below_wall_count = density_idx == last_idx ? min_skin_below_wall_count : 0;
            const coord_t overlap = infill_overlap - (density_idx == last_idx ? 0 : wall_line_count * infill_line_width);

            generate_infill(infill_below_skin, infill_line_distance_here, overlap, infill_shift, skin_below_wall_count, density_idx, use_endpieces, patterns);

            if (density_idx < last_idx)
            {
                const coord_t cut_offset = get_cut_offset(zig_zaggify_infill, infill_line_width, min_skin_below_wall_count);
                Shape tool = infill_below_skin.offset(static_cast<int>(cut_offset));
                infill_lines_here = tool.intersection(infill_lines_here);
            }
            patterns.lines.push_back(infill_lines_here);
            // normal processing for the infill that isn't below skin
            in_outline = infill_not_below_skin;
            if (density_idx == last_idx)
            {
                sparse_in_outline = infill_not_below_skin;
            }
        }

        const coord_t circumference = in_outline.length();
        // Originally an area of 0.4*0.4*2 (2 line width squares) was found to be a good threshold for removal.
        // However we found that this doesn't scale well with polygons with larger circumference (https://github.com/Ultimaker/Cura/issues/3992).
        // Given that the original test worked for approximately 2x2cm models, this scaling by circumference should make it work for any size.
        constexpr double minimum_small_area_factor = 0.4 * 0.4 / 40000;
        const double minimum_small_area = minimum_small_area_factor * circumference;

        // This is only for density infill, because after generating the infill might appear unnecessary infill on walls
        // especially on vertical surfaces
        in_outline.removeSmallAreas(minimum_small_area);

        constexpr size_t wall_line_count_here = 0; // Wall toolpaths were generated in generateGradualInfill for the sparsest density, denser parts don't have walls by default
        const coord_t overlap = settings.get<coord_t>("infill_overlap_mm");

        generate_infill(in_outline, infill_line_distance_here, overlap, infill_shift, wall_line_count_here, density_idx, use_endpieces, patterns);

        if (density_idx < last_idx)
        {
            const coord_t cut_offset = get_cut_offset(zig_zaggify_infill, infill_line_width, wall_line_count);
            Shape tool = sparse_in_outline.offset(static_cast<int>(cut_offset));
            infill_lines_here = tool.intersection(infill_lines_here);
        }
        patterns.lines.push_back(infill_lines_here);
        patterns.polygons.push_back(infill_polygons_here);
    }

    // The extra infill walls were generated separately. Add these too.
    patterns.toolpaths.insert(patterns.toolpaths.end(), part.infill_wall_toolpaths.begin(), part.infill_wall_toolpaths.end());

    if (settings.get<coord_t>("wall_line_count") // Disable feature if no walls - it can leave dangling lines at edges
        && infill_pattern != EFillMethod::LIGHTNING // Lightning doesn't make enclosed regions
        && infill_pattern != EFillMethod::CONCENTRIC // Doesn't handle 'holes' in infill lines very well
        && infill_pattern != EFillMethod::CROSS // Ditto
        && infill_pattern != EFillMethod::CROSS_3D) // Ditto
    {
        // addExtraLinesToSupportSurfacesAbove(infill_lines, infill_polygons, wall_tool_paths, part, infill_line_width, gcode_layer, mesh);
    }
}

void MeshInfillGenerator::processSingleLayerInfill(
    const SliceLayerPart& part,
    const size_t last_idx,
    const EFillMethod infill_pattern,
    const size_t combine_idx,
    const coord_t infill_overlap,
    const auto generate_infill) const
{
    Infill::GeneratedPatterns patterns;
    patterns.toolpaths = part.infill_wall_toolpaths;
    for (size_t density_idx = last_idx; (int)density_idx >= 0; density_idx--)
    { // combine different density infill areas (for gradual infill)
        size_t density_factor = 2 << density_idx; // == pow(2, density_idx + 1)
        coord_t infill_line_distance_here = infill_line_distance_ * density_factor; // the highest density infill combines with the next to create a grid with density_factor
        const coord_t infill_shift = infill_line_distance_here / 2;
        if (density_idx == part.infill_area_per_combine_per_density.size() - 1 || infill_pattern == EFillMethod::CROSS || infill_pattern == EFillMethod::CROSS_3D)
        {
            infill_line_distance_here /= 2;
        }

        constexpr size_t wall_line_count = 0; // wall toolpaths are when gradual infill areas are determined
        constexpr bool use_endpieces = true;

        generate_infill(
            part.infill_area_per_combine_per_density[density_idx][combine_idx],
            infill_line_distance_here,
            infill_overlap,
            infill_shift,
            wall_line_count,
            density_idx,
            use_endpieces,
            patterns);
    }
}

bool MeshInfillGenerator::partitionInfillBySkinAbove(
    Shape& infill_below_skin,
    Shape& infill_not_below_skin,
    const LayerPlanPtr& layer_plan,
    const SliceLayerPart& part,
    coord_t infill_line_width) const
{
    constexpr coord_t tiny_infill_offset = 20;
    const auto skin_edge_support_layers = getMesh()->settings.get<size_t>("skin_edge_support_layers");
    Shape skin_above_combined; // skin regions on the layers above combined with small gaps between

    // working from the highest layer downwards, combine the regions of skin on all the layers
    // but don't let the regions merge together
    // otherwise "terraced" skin regions on separate layers will look like a single region of unbroken skin
    for (size_t i = skin_edge_support_layers; i > 0; --i)
    {
        const size_t skin_layer_nr = layer_plan->getLayerIndex() + i;
        if (skin_layer_nr < getMesh()->layers.size())
        {
            for (const SliceLayerPart& part_i : getMesh()->layers[skin_layer_nr].parts)
            {
                for (const SkinPart& skin_part : part_i.skin_parts)
                {
                    // Limit considered areas to the ones that should have infill underneath at the current layer.
                    const Shape relevant_outline = skin_part.outline.intersection(part.getOwnInfillArea());

                    if (! skin_above_combined.empty())
                    {
                        // does this skin part overlap with any of the skin parts on the layers above?
                        const Shape overlap = skin_above_combined.intersection(relevant_outline);
                        if (! overlap.empty())
                        {
                            // yes, it overlaps, need to leave a gap between this skin part and the others
                            if (i > 1) // this layer is the 2nd or higher layer above the layer whose infill we're printing
                            {
                                // looking from the side, if the combined regions so far look like this...
                                //
                                //     ----------------------------------
                                //
                                // and the new skin part looks like this...
                                //
                                //             -------------------------------------
                                //
                                // the result should be like this...
                                //
                                //     ------- -------------------------- ----------

                                // expand the overlap region slightly to make a small gap
                                const Shape overlap_expanded = overlap.offset(tiny_infill_offset);
                                // subtract the expanded overlap region from the regions accumulated from higher layers
                                skin_above_combined = skin_above_combined.difference(overlap_expanded);
                                // subtract the expanded overlap region from this skin part and add the remainder to the overlap region
                                skin_above_combined.push_back(relevant_outline.difference(overlap_expanded));
                                // and add the overlap area as well
                                skin_above_combined.push_back(overlap);
                            }
                            else // this layer is the 1st layer above the layer whose infill we're printing
                            {
                                // add this layer's skin region without subtracting the overlap but still make a gap between this skin region and what has been accumulated so
                                // far we do this so that these skin region edges will definitely have infill walls below them

                                // looking from the side, if the combined regions so far look like this...
                                //
                                //     ----------------------------------
                                //
                                // and the new skin part looks like this...
                                //
                                //             -------------------------------------
                                //
                                // the result should be like this...
                                //
                                //     ------- -------------------------------------

                                skin_above_combined = skin_above_combined.difference(relevant_outline.offset(tiny_infill_offset));
                                skin_above_combined.push_back(relevant_outline);
                            }
                        }
                        else // no overlap
                        {
                            skin_above_combined.push_back(relevant_outline);
                        }
                    }
                    else // this is the first skin region we have looked at
                    {
                        skin_above_combined.push_back(relevant_outline);
                    }
                }
            }
        }

        // the shrink/expand here is to remove regions of infill below skin that are narrower than the width of the infill walls otherwise the infill walls could merge and form
        // a bump
        infill_below_skin = skin_above_combined.intersection(part.infill_area_per_combine_per_density.back().front()).offset(-infill_line_width).offset(infill_line_width);

        constexpr bool remove_small_holes_from_infill_below_skin = true;
        constexpr double min_area_multiplier = 25;
        const double min_area = INT2MM(infill_line_width) * INT2MM(infill_line_width) * min_area_multiplier;
        infill_below_skin.removeSmallAreas(min_area, remove_small_holes_from_infill_below_skin);

        // there is infill below skin, is there also infill that isn't below skin?
        infill_not_below_skin = part.infill_area_per_combine_per_density.back().front().difference(infill_below_skin);
        infill_not_below_skin.removeSmallAreas(min_area);
    }

    // need to take skin/infill overlap that was added in SkinInfillAreaComputation::generateInfill() into account
    const coord_t infill_skin_overlap = getMesh()->settings.get<coord_t>((part.wall_toolpaths.size() > 1) ? "wall_line_width_x" : "wall_line_width_0") / 2;
    const Shape infill_below_skin_overlap = infill_below_skin.offset(-(infill_skin_overlap + tiny_infill_offset));

    return ! infill_below_skin_overlap.empty() && ! infill_not_below_skin.empty();
}

} // namespace cura