// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include <algorithm> //For std::partition_copy.

#include "WallToolPaths.h"

#include "SkeletalTrapezoidation.h"
#include "utils/polygonUtils.h"

namespace cura
{
constexpr coord_t transition_length_multiplier = 2;

WallToolPaths::WallToolPaths(const Polygons& outline, const coord_t nominal_bead_width, const coord_t inset_count,
                             const Settings& settings)
    : outline(outline)
    , bead_width_0(nominal_bead_width)
    , bead_width_x(nominal_bead_width)
    , inset_count(inset_count)
    , strategy_type(settings.get<StrategyType>("beading_strategy_type"))
    , print_thin_walls(settings.get<bool>("fill_outline_gaps"))
    , min_feature_size(print_thin_walls ? new coord_t(settings.get<coord_t>("min_feature_size")) : nullptr)
    , min_bead_width(print_thin_walls ? new coord_t(settings.get<coord_t>("min_bead_width")) : nullptr)
    , small_area_length(INT2MM(static_cast<double>(nominal_bead_width) / 2))
    , transition_length(transition_length_multiplier * nominal_bead_width)
    , toolpaths_generated(false)
{
}

WallToolPaths::WallToolPaths(const Polygons& outline, const coord_t bead_width_0, const coord_t bead_width_x,
                             const coord_t inset_count, const Settings& settings)
    : outline(outline)
    , bead_width_0(bead_width_0)
    , bead_width_x(bead_width_x)
    , inset_count(inset_count)
    , strategy_type(settings.get<StrategyType>("beading_strategy_type"))
    , print_thin_walls(settings.get<bool>("fill_outline_gaps"))
    , min_feature_size(print_thin_walls ? new coord_t(settings.get<coord_t>("min_feature_size")) : nullptr)
    , min_bead_width(print_thin_walls ? new coord_t(settings.get<coord_t>("min_bead_width")) : nullptr)
    , small_area_length(INT2MM(static_cast<double>(bead_width_0) / 2))
    , transition_length(transition_length_multiplier * bead_width_0)
    , toolpaths_generated(false)
{
}

const VariableWidthPaths& WallToolPaths::generate()
{
    assert(inset_count > 0 && "inset count should be more than 0");
    constexpr coord_t smallest_segment = 50;
    constexpr coord_t allowed_distance = 50;
    constexpr coord_t epsilon_offset = (allowed_distance / 2) - 1;
    constexpr float transitioning_angle = 0.5;

    // Simplify outline for boost::voronoi consumption. Absolutely no self intersections or near-self intersections allowed:
    // TODO: Open question: Does this indeed fix all (or all-but-one-in-a-million) cases for manifold but otherwise possibly complex polygons?
    Polygons prepared_outline = outline.offset(-epsilon_offset).offset(epsilon_offset);
    prepared_outline.simplify(smallest_segment, allowed_distance);
    PolygonUtils::fixSelfIntersections(epsilon_offset, prepared_outline);
    prepared_outline.removeDegenerateVerts();
    prepared_outline.removeColinearEdges();
    prepared_outline.removeSmallAreas(small_area_length * small_area_length, false);

    if (prepared_outline.area() > 0)
    {
        const coord_t max_bead_count = 2 * inset_count;
        const auto beading_strat = std::unique_ptr<BeadingStrategy>(BeadingStrategyFactory::makeStrategy(
            strategy_type, bead_width_0, bead_width_x, transition_length, transitioning_angle, min_bead_width,
            min_feature_size, max_bead_count));
        SkeletalTrapezoidation wall_maker(prepared_outline, *beading_strat, beading_strat->transitioning_angle);
        wall_maker.generateToolpaths(toolpaths);
        computeInnerContour();
    }
    removeEmptyToolPaths(toolpaths);
    toolpaths_generated = true;
    return toolpaths;
}

const VariableWidthPaths& WallToolPaths::getToolPaths()
{
    if (!toolpaths_generated)
    {
        return generate();
    }
    return toolpaths;
}

void WallToolPaths::computeInnerContour()
{
    //We'll remove all 0-width paths from the original toolpaths and store them separately as polygons.
    VariableWidthPaths actual_toolpaths;
    actual_toolpaths.reserve(toolpaths.size()); //A bit too much, but the correct order of magnitude.
    VariableWidthPaths contour_paths;
    contour_paths.reserve(toolpaths.size() / inset_count);
    std::partition_copy(toolpaths.begin(), toolpaths.end(), std::back_inserter(actual_toolpaths), std::back_inserter(contour_paths),
        [](const VariableWidthLines& path)
        {
            for(const ExtrusionLine& line : path)
            {
                for(const ExtrusionJunction& junction : line.junctions)
                {
                    return junction.w != 0; //On the first actual junction, decide: If it's got 0 width, this is a contour. Otherwise it is an actual toolpath.
                }
            }
            return true; //No junctions with any vertices? Classify it as a toolpath then.
        });
    toolpaths = actual_toolpaths; //Filtered out the 0-width paths.

    //Now convert the contour_paths to Polygons to denote the inner contour of the walled areas.
    inner_contour.clear();
    for(const VariableWidthLines& path : contour_paths)
    {
        for(const ExtrusionLine& line : path)
        {
            inner_contour.emplace_back();
            for(const ExtrusionJunction& junction : line.junctions)
            {
                inner_contour.back().add(junction.p);
            }
        }
    }
}

const Polygons& WallToolPaths::getInnerContour()
{
    if (!toolpaths_generated && inset_count > 0)
    {
        generate();
    }
    return inner_contour;
}

const Polygons& WallToolPaths::getOutline() const
{
    return outline;
}

Polygons WallToolPaths::innerContourFromToolpaths(const VariableWidthPaths& toolpaths)
{
    // TODO: CURA-7681
    return Polygons();
}

bool WallToolPaths::removeEmptyToolPaths(VariableWidthPaths& toolpaths)
{
    toolpaths.erase(std::remove_if(toolpaths.begin(), toolpaths.end(), [](const VariableWidthLines& lines)
                                   {
                                       return lines.empty();
                                   }), toolpaths.end());
    return toolpaths.empty();
}


} // namespace cura
