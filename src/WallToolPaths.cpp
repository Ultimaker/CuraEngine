// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include <algorithm> //For std::partition_copy and std::min_element.
#include <unordered_set>

#include "WallToolPaths.h"

#include "SkeletalTrapezoidation.h"
#include "BeadingStrategy/BeadingOrderOptimizer.h"
#include "utils/SparsePointGrid.h" //To stitch the inner contour.
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
    , min_feature_size(settings.get<coord_t>("min_feature_size"))
    , min_bead_width(settings.get<coord_t>("min_bead_width"))
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
    , min_feature_size(settings.get<coord_t>("min_feature_size"))
    , min_bead_width(settings.get<coord_t>("min_bead_width"))
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
            strategy_type, bead_width_0, bead_width_x, transition_length, transitioning_angle, print_thin_walls, min_bead_width,
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

    //We're going to have to stitch these paths since not all walls may be closed contours.
    //Since these walls have 0 width they should theoretically be closed. But there may be rounding errors.
    //Create a bucket grid to find endpoints that are close together.
    const coord_t minimum_line_width = bead_width_0 / 2;
    struct ExtrusionLineEndpointLocator
    {
        Point operator()(ExtrusionLine* line)
        {
            return Point(line->junctions.front().p);
        }
    };
    SparsePointGrid<ExtrusionLine*, ExtrusionLineEndpointLocator> line_endpoints(minimum_line_width); //Only find endpoints closer than minimum_line_width, so we can't ever accidentally make crossing contours.
    for(VariableWidthLines& path : contour_paths)
    {
        for(ExtrusionLine& line : path)
        {
            line_endpoints.insert(&line);
        }
    }
    //Then go through all lines and construct chains of polylines if the endpoints are nearby.
    std::unordered_set<ExtrusionLine*> processed_lines; //Track which lines were already processed to not process them twice.
    for(VariableWidthLines& path : contour_paths)
    {
        for(ExtrusionLine& line : path)
        {
            if(processed_lines.find(&line) != processed_lines.end()) //We already added this line before.
            {
                continue;
            }
            inner_contour.emplace_back();
            ExtrusionLine* nearest = &line;
            while(nearest)
            {
                if(processed_lines.find(nearest) != processed_lines.end())
                {
                    break; //Looping. This contour is already processed.
                }

                for(const ExtrusionJunction& junction : nearest->junctions)
                {
                    inner_contour.back().add(junction.p);
                }
                processed_lines.insert(nearest);

                //Find any nearby lines to attach.
                const Point current_position = nearest->junctions.back().p;
                const std::vector<ExtrusionLine*> nearby = line_endpoints.getNearby(current_position, minimum_line_width);
                nearest = nullptr;
                coord_t nearest_dist2 = std::numeric_limits<coord_t>::max();
                for(ExtrusionLine* candidate : nearby)
                {
                    if(processed_lines.find(candidate) != processed_lines.end())
                    {
                        continue; //Already processed this line before. It's linked to something else.
                    }
                    
                    const coord_t dist2 = vSize2(candidate->junctions.front().p - current_position);
                    if(dist2 < nearest_dist2)
                    {
                        nearest = candidate;
                        nearest_dist2 = dist2;
                        continue;
                    }
                }
            }
        }
    }

    //The output walls from the skeletal trapezoidation have no known winding order, especially if they are joined together from polylines.
    //They can be in any direction, clockwise or counter-clockwise, regardless of whether the shapes are positive or negative.
    //To get a correct shape, we need to make the outside contour positive and any holes inside negative.
    //This can be done by applying the even-odd rule to the shape. This rule is not sensitive to the winding order of the polygon.
    //The even-odd rule would be incorrect if the polygon self-intersects, but that should never be generated by the skeletal trapezoidation.
    inner_contour = inner_contour.unionPolygons(Polygons(), ClipperLib::pftEvenOdd);
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

bool WallToolPaths::removeEmptyToolPaths(VariableWidthPaths& toolpaths)
{
    toolpaths.erase(std::remove_if(toolpaths.begin(), toolpaths.end(), [](const VariableWidthLines& lines)
                                   {
                                       return lines.empty();
                                   }), toolpaths.end());
    return toolpaths.empty();
}


} // namespace cura
