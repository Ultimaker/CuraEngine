// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "WallToolPaths.h"

#include "BeadingStrategy/BeadingStrategyFactory.h"
#include "SkeletalTrapezoidation.h"
#include "utils/polygonUtils.h"

namespace cura
{
constexpr coord_t transition_length_multiplier = 2;

WallToolPaths::WallToolPaths(const Polygons& outline, const coord_t nominal_bead_width, const size_t inset_count,
                             const Settings& settings)
    : outline(outline)
    , nominal_bead_width(nominal_bead_width)
    , inset_count(inset_count)
    , strategy_type(settings.get<StrategyType>("beading_strategy_type"))
    , widening_beading_enabled(settings.get<bool>("widening_beading_enabled"))
    , min_bead_width(widening_beading_enabled ? new coord_t(settings.get<coord_t>("min_bead_width")) : nullptr)
    , min_feature_size(widening_beading_enabled ? new coord_t(settings.get<coord_t>("min_feature_size")) : nullptr)
    , small_area_length(INT2MM(static_cast<double>(nominal_bead_width) / 2))
    , transition_length(transition_length_multiplier * nominal_bead_width)
    , toolpaths_generated(false)
{
}

const ToolPaths& WallToolPaths::generate()
{
    constexpr coord_t smallest_segment = 50;
    constexpr coord_t allowed_distance = 50;
    constexpr coord_t epsilon_offset = (allowed_distance / 2) - 1;
    // TODO: after we ironed out all the bugs, remove-colinear should go.
    constexpr float max_colinear_angle = 0.03; // Way too large
    constexpr float transitioning_angle = 0.5;

    Polygons prepared_outline = outline.offset(-epsilon_offset).offset(epsilon_offset);
    prepared_outline.simplify(smallest_segment, allowed_distance);
    PolygonUtils::fixSelfIntersections(epsilon_offset, prepared_outline);
    prepared_outline.removeDegenerateVerts();
    prepared_outline.removeColinearEdges();
    // TODO: complete guess as to when arachne starts breaking, but it doesn't  function well when an area is really small apearantly?
    prepared_outline.removeSmallAreas(small_area_length * small_area_length, false);

    if (prepared_outline.area() > 0)
    {
        const auto beading_strat = std::unique_ptr<BeadingStrategy>(BeadingStrategyFactory::makeStrategy(
            strategy_type, nominal_bead_width, transition_length, transitioning_angle, min_bead_width, min_feature_size,
            2 * inset_count)); // TODO: deal with beading-strats & (their) magic parameters
        SkeletalTrapezoidation wall_maker(prepared_outline, *beading_strat, beading_strat->transitioning_angle);
        wall_maker.generateToolpaths(toolpaths);
    }
    return toolpaths;
}

const Polygons& WallToolPaths::getInnerContour()
{
    if (!toolpaths_generated)
    {
        generate();
    }
    // TODO: CURA-7681  -> inner_contour = innerContourFromToolpaths(toolpaths);
    // TODO: Check to make sure if this "correctly generated for now"
    inner_contour = outline.offset(-nominal_bead_width * inset_count);
    return inner_contour;
}

const Polygons& WallToolPaths::getOutline() const
{
    return outline;
}

const BinJunctions& WallToolPaths::getBinJunctions()
{
    if (!toolpaths_generated)
    {
        generate();
    }
    binJunctions = toolPathsToBinJunctions(toolpaths, inset_count);
    return binJunctions;
}

BinJunctions WallToolPaths::toolPathsToBinJunctions(const ToolPaths& toolpaths, coord_t num_insets)
{
    BinJunctions insets(
        num_insets); // Vector of insets (bins). Each inset is a vector of paths. Each path is a vector of lines.
    for (const std::list<ExtrusionLine>& path : toolpaths)
    {
        if (path.empty()) // Don't bother printing these.
        {
            continue;
        }
        const size_t inset_index = path.front().inset_idx;

        // Convert list of extrusion lines to vectors of extrusion junctions, and add those to the binned insets.
        for (const ExtrusionLine& line : path)
        {
            insets[inset_index].emplace_back(line.junctions.begin(), line.junctions.end());
        }
    }
    return insets;
}

Polygons WallToolPaths::innerContourFromToolpaths(const ToolPaths& toolpaths)
{
    // TODO: CURA-7681
    return Polygons();
}

} // namespace cura
