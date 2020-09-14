// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "WallToolPaths.h"

#include "SkeletalTrapezoidation.h"
#include "utils/polygonUtils.h"

namespace cura
{
constexpr coord_t transition_length_multiplier = 2;

WallToolPaths::WallToolPaths(const Polygons& outline, const coord_t nominal_bead_width, const coord_t inset_count,
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

const VariableWidthPath& WallToolPaths::generate()
{
    assert(("inset count should be more then 0", inset_count > 0));
    constexpr coord_t smallest_segment = 50;
    constexpr coord_t allowed_distance = 50;
    constexpr coord_t epsilon_offset = (allowed_distance / 2) - 1;
    constexpr float transitioning_angle = 0.5;

    Polygons prepared_outline = outline.offset(-epsilon_offset).offset(epsilon_offset);
    prepared_outline.simplify(smallest_segment, allowed_distance);
    PolygonUtils::fixSelfIntersections(epsilon_offset, prepared_outline);
    prepared_outline.removeDegenerateVerts();
    prepared_outline.removeColinearEdges();
    // TODO: complete guess as to when arachne starts breaking, but it doesn't  function well when an area is really
    // small apparently?
    prepared_outline.removeSmallAreas(small_area_length * small_area_length, false);

    if (prepared_outline.area() > 0)
    {
        const coord_t max_bead_count = 2 * inset_count;
        const auto beading_strat = std::unique_ptr<BeadingStrategy>(BeadingStrategyFactory::makeStrategy(
            strategy_type, nominal_bead_width, transition_length, transitioning_angle, min_bead_width, min_feature_size,
            max_bead_count)); // TODO: deal with beading-strats & (their) magic parameters
        SkeletalTrapezoidation wall_maker(prepared_outline, *beading_strat, beading_strat->transitioning_angle);
        wall_maker.generateToolpaths(toolpaths);
    }
    toolpaths_generated = true;
    return toolpaths;
}

const VariableWidthPath& WallToolPaths::getToolPaths()
{
    if (!toolpaths_generated)
    {
        return generate();
    }
    return toolpaths;
}

const Polygons& WallToolPaths::getInnerContour()
{
    if (!toolpaths_generated && inset_count > 0)
    {
        generate();
    }
    else
    {
        return outline;
    }
    // TODO: CURA-7681  -> inner_contour = innerContourFromToolpaths(toolpaths);
    // TODO: Check to make sure if this "correctly generated for now"
    if (inner_contour.empty())
    {
        const coord_t offset_distance = nominal_bead_width * inset_count;
        inner_contour = outline.offset(-static_cast<int>(offset_distance));
        inner_contour.simplify();
    }
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

BinJunctions WallToolPaths::toolPathsToBinJunctions(const VariableWidthPath& toolpaths, const coord_t num_insets)
{
    // Vector of insets (bins). Each inset is a vector of paths. Each path is a vector of lines.
    BinJunctions insets(static_cast<size_t>(num_insets));
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

Polygons WallToolPaths::innerContourFromToolpaths(const VariableWidthPath& toolpaths)
{
    // TODO: CURA-7681
    return Polygons();
}

} // namespace cura
