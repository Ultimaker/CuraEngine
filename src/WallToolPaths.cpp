// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include <algorithm> //For std::partition_copy and std::min_element.
#include <unordered_set>

#include "WallToolPaths.h"

#include "SkeletalTrapezoidation.h"
#include "utils/SparsePointGrid.h" //To stitch the inner contour.
#include "utils/polygonUtils.h"
#include "ExtruderTrain.h"
#include "utils/PolylineStitcher.h"

namespace cura
{

WallToolPaths::WallToolPaths(const Polygons& outline, const coord_t nominal_bead_width, const size_t inset_count, const coord_t wall_0_inset,
                             const Settings& settings)
    : outline(outline)
    , bead_width_0(nominal_bead_width)
    , bead_width_x(nominal_bead_width)
    , inset_count(inset_count)
    , wall_0_inset(wall_0_inset)
    , strategy_type(settings.get<StrategyType>("beading_strategy_type"))
    , print_thin_walls(settings.get<bool>("fill_outline_gaps"))
    , min_feature_size(settings.get<coord_t>("min_feature_size"))
    , min_bead_width(settings.get<coord_t>("min_bead_width"))
    , small_area_length(INT2MM(static_cast<double>(nominal_bead_width) / 2))
    , toolpaths_generated(false)
    , settings(settings)
{
}

WallToolPaths::WallToolPaths(const Polygons& outline, const coord_t bead_width_0, const coord_t bead_width_x,
                             const size_t inset_count, const coord_t wall_0_inset, const Settings& settings)
    : outline(outline)
    , bead_width_0(bead_width_0)
    , bead_width_x(bead_width_x)
    , inset_count(inset_count)
    , wall_0_inset(wall_0_inset)
    , strategy_type(settings.get<StrategyType>("beading_strategy_type"))
    , print_thin_walls(settings.get<bool>("fill_outline_gaps"))
    , min_feature_size(settings.get<coord_t>("min_feature_size"))
    , min_bead_width(settings.get<coord_t>("min_bead_width"))
    , small_area_length(INT2MM(static_cast<double>(bead_width_0) / 2))
    , toolpaths_generated(false)
    , settings(settings)
{
}

const VariableWidthPaths& WallToolPaths::generate()
{
    const coord_t smallest_segment = settings.get<coord_t>("meshfix_maximum_resolution");
    const coord_t allowed_distance = settings.get<coord_t>("meshfix_maximum_deviation");
    const coord_t epsilon_offset = (allowed_distance / 2) - 1;
    const AngleRadians transitioning_angle = settings.get<AngleRadians>("wall_transition_angle");
    constexpr coord_t discretization_step_size = MM2INT(0.8);

    // Simplify outline for boost::voronoi consumption. Absolutely no self intersections or near-self intersections allowed:
    // TODO: Open question: Does this indeed fix all (or all-but-one-in-a-million) cases for manifold but otherwise possibly complex polygons?
    Polygons prepared_outline = outline.offset(-epsilon_offset).offset(epsilon_offset * 2).offset(-epsilon_offset);
    prepared_outline.simplify(smallest_segment, allowed_distance);
    PolygonUtils::fixSelfIntersections(epsilon_offset, prepared_outline);
    prepared_outline.removeDegenerateVerts();
    prepared_outline.removeColinearEdges(AngleRadians(0.005));
    // Removing collinear edges may introduce self intersections, so we need to fix them again
    PolygonUtils::fixSelfIntersections(epsilon_offset, prepared_outline);
    prepared_outline.removeDegenerateVerts();
    prepared_outline.removeSmallAreas(small_area_length * small_area_length, false);

    if (prepared_outline.area() > 0)
    {
        const coord_t wall_transition_length = settings.get<coord_t>("wall_transition_length");
        const Ratio wall_split_middle_threshold = settings.get<Ratio>("wall_split_middle_threshold");  // For an uneven nr. of lines: When to split the middle wall into two.
        const Ratio wall_add_middle_threshold = settings.get<Ratio>("wall_add_middle_threshold");      // For an even nr. of lines: When to add a new middle in between the innermost two walls.
        const int wall_distribution_count = settings.get<int>("wall_distribution_count");
        const size_t max_bead_count = (inset_count < std::numeric_limits<coord_t>::max() / 2) ? 2 * inset_count : std::numeric_limits<coord_t>::max();
        const auto beading_strat = BeadingStrategyFactory::makeStrategy
            (
                strategy_type,
                bead_width_0,
                bead_width_x,
                wall_transition_length,
                transitioning_angle,
                print_thin_walls,
                min_bead_width,
                min_feature_size,
                wall_split_middle_threshold,
                wall_add_middle_threshold,
                max_bead_count,
                wall_0_inset,
                wall_distribution_count
            );
        const coord_t transition_filter_dist = settings.get<coord_t>("wall_transition_filter_distance");
        SkeletalTrapezoidation wall_maker
        (
            prepared_outline,
            *beading_strat,
            beading_strat->getTransitioningAngle(),
            discretization_step_size,
            transition_filter_dist,
            wall_transition_length
        );
        wall_maker.generateToolpaths(toolpaths);
        computeInnerContour();
    }
    
    stitchToolPaths(toolpaths, settings);
    
    simplifyToolPaths(toolpaths, settings);

    removeEmptyToolPaths(toolpaths);
    assert(std::is_sorted(toolpaths.cbegin(), toolpaths.cend(),
                          [](const VariableWidthLines& l, const VariableWidthLines& r)
                          {
                              return l.front().inset_idx < r.front().inset_idx;
                          }) && "WallToolPaths should be sorted from the outer 0th to inner_walls");
    toolpaths_generated = true;
    return toolpaths;
}


void WallToolPaths::stitchToolPaths(VariableWidthPaths& toolpaths, const Settings& settings)
{
    const coord_t stitch_distance = settings.get<coord_t>("wall_line_width_x") / 4;

    // separate polygonal wall lines and gap filling odd lines and stitch the polygonal lines into closed polygons
    const size_t inset_count = toolpaths.size();
    for (unsigned int wall_idx = 0; wall_idx < inset_count; wall_idx++)
    {
        VariableWidthLines& wall_lines = toolpaths[wall_idx];
        for (ExtrusionLine& line : wall_lines)
        {
            assert(line.inset_idx == wall_idx);
        }
        
        VariableWidthLines stitched_polylines;
        VariableWidthLines closed_polygons;
        PolylineStitcher<VariableWidthLines, ExtrusionLine, ExtrusionJunction>::stitch(wall_lines, stitched_polylines, closed_polygons, stitch_distance);
        wall_lines = stitched_polylines;

        for (ExtrusionLine& wall_polygon : closed_polygons)
        {
            if (wall_polygon.junctions.empty())
            {
                continue;
            }
            if (wall_polygon.junctions.size() > 1)
            {
                wall_polygon.junctions.emplace_back(wall_polygon.junctions.front()); // Make polygon back into polyline. TODO: make it possibleto simplify polygonal toolpaths and don't convert from polygons backto polylines here
            }
            wall_lines.emplace_back(std::move(wall_polygon));
        }

        for (ExtrusionLine& line : wall_lines)
        {
            line.inset_idx = wall_idx;
        }
    }
}

void WallToolPaths::simplifyToolPaths(VariableWidthPaths& toolpaths, const Settings& settings)
{
    for (size_t toolpaths_idx = 0; toolpaths_idx < toolpaths.size(); ++toolpaths_idx)
    {
        const coord_t maximum_resolution = settings.get<coord_t>("meshfix_maximum_resolution");
        const coord_t maximum_deviation = settings.get<coord_t>("meshfix_maximum_deviation");
        const coord_t maximum_extrusion_area_deviation = settings.get<int>("meshfix_maximum_extrusion_area_deviation"); // unit: μm²
        for (auto& line : toolpaths[toolpaths_idx])
        {
            line.simplify(maximum_resolution * maximum_resolution, maximum_deviation * maximum_deviation, maximum_extrusion_area_deviation);
        }
    }
}

const VariableWidthPaths& WallToolPaths::getToolPaths()
{
    if (!toolpaths_generated)
    {
        return generate();
    }
    return toolpaths;
}

void WallToolPaths::pushToolPaths(VariableWidthPaths& paths)
{
    if (! toolpaths_generated)
    {
        generate();
    }
    paths.insert(paths.end(), toolpaths.begin(), toolpaths.end());
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
    if (! actual_toolpaths.empty())
    {
        toolpaths = std::move(actual_toolpaths); //Filtered out the 0-width paths.
    }
    else
    {
        toolpaths.clear();
    }

    //Now convert the contour_paths to Polygons to denote the inner contour of the walled areas.
    inner_contour.clear();

    //We're going to have to stitch these paths since not all walls may be closed contours.
    //Since these walls have 0 width they should theoretically be closed. But there may be rounding errors.
    const coord_t minimum_line_width = bead_width_0 / 2;
    stitchContours(contour_paths, minimum_line_width, inner_contour);

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
    else if(inset_count == 0)
    {
        return outline;
    }
    return inner_contour;
}

bool WallToolPaths::removeEmptyToolPaths(VariableWidthPaths& toolpaths)
{
    toolpaths.erase(std::remove_if(toolpaths.begin(), toolpaths.end(), [](const VariableWidthLines& lines)
                                   {
                                       return lines.empty();
                                   }), toolpaths.end());
    return toolpaths.empty();
}

void WallToolPaths::stitchContours(const VariableWidthPaths& input, const coord_t stitch_distance, Polygons& output)
{
    //Create a bucket grid to find endpoints that are close together.
    struct ExtrusionLineStartLocator
    {
        Point operator()(const ExtrusionLine* line)
        {
            return Point(line->junctions.front().p);
        }
    };
    struct ExtrusionLineEndLocator
    {
        Point operator()(const ExtrusionLine* line)
        {
            return Point(line->junctions.back().p);
        }
    };
    SparsePointGrid<const ExtrusionLine*, ExtrusionLineStartLocator> line_starts(stitch_distance); //Only find endpoints closer than minimum_line_width, so we can't ever accidentally make crossing contours.
    SparsePointGrid<const ExtrusionLine*, ExtrusionLineEndLocator> line_ends(stitch_distance);
    for(const VariableWidthLines& path : input)
    {
        for(const ExtrusionLine& line : path)
        {
            line_starts.insert(&line);
            line_ends.insert(&line);
        }
    }
    //Then go through all lines and construct chains of polylines if the endpoints are nearby.
    std::unordered_set<const ExtrusionLine*> processed_lines; //Track which lines were already processed to not process them twice.
    for(const VariableWidthLines& path : input)
    {
        for(const ExtrusionLine& line : path)
        {
            if(processed_lines.find(&line) != processed_lines.end()) //We already added this line before. It got added as a nearby line.
            {
                continue;
            }
            //We'll create a chain of polylines that get joined together. We can add polylines on both ends!
            std::deque<const ExtrusionLine*> chain;
            std::deque<bool> is_reversed; //Lines could need to be inserted in reverse. Must coincide with the `chain` deque.
            const ExtrusionLine* nearest = &line; //At every iteration, add the polyline that joins together most closely.
            bool nearest_reverse = false; //Whether the next line to insert must be inserted in reverse.
            bool nearest_before = false; //Whether the next line to insert must be inserted in the front of the chain.
            while(nearest)
            {
                if(processed_lines.find(nearest) != processed_lines.end())
                {
                    break; //Looping. This contour is already processed.
                }
                processed_lines.insert(nearest);
                if(nearest_before)
                {
                    chain.push_front(nearest);
                    is_reversed.push_front(nearest_reverse);
                }
                else
                {
                    chain.push_back(nearest);
                    is_reversed.push_back(nearest_reverse);
                }

                //Find any nearby lines to attach. Look on both ends of our current chain and find both ends of polylines.
                const Point chain_start = is_reversed.front() ? chain.front()->junctions.back().p : chain.front()->junctions.front().p;
                const Point chain_end = is_reversed.back() ? chain.back()->junctions.front().p : chain.back()->junctions.back().p;
                std::vector<const ExtrusionLine*> starts_near_start = line_starts.getNearby(chain_start, stitch_distance);
                std::vector<const ExtrusionLine*> ends_near_start = line_ends.getNearby(chain_start, stitch_distance);
                std::vector<const ExtrusionLine*> starts_near_end = line_starts.getNearby(chain_end, stitch_distance);
                std::vector<const ExtrusionLine*> ends_near_end = line_ends.getNearby(chain_end, stitch_distance);

                nearest = nullptr;
                coord_t nearest_dist2 = std::numeric_limits<coord_t>::max();
                for(const ExtrusionLine* candidate : starts_near_start)
                {
                    if(processed_lines.find(candidate) != processed_lines.end())
                    {
                        continue; //Already processed this line before. It's linked to something else.
                    }
                    const coord_t dist2 = vSize2(candidate->junctions.front().p - chain_start);
                    if(dist2 < nearest_dist2)
                    {
                        nearest = candidate;
                        nearest_dist2 = dist2;
                        nearest_reverse = true;
                        nearest_before = true;
                    }
                }
                for(const ExtrusionLine* candidate : ends_near_start)
                {
                    if(processed_lines.find(candidate) != processed_lines.end())
                    {
                        continue;
                    }
                    const coord_t dist2 = vSize2(candidate->junctions.back().p - chain_start);
                    if(dist2 < nearest_dist2)
                    {
                        nearest = candidate;
                        nearest_dist2 = dist2;
                        nearest_reverse = false;
                        nearest_before = true;
                    }
                }
                for(const ExtrusionLine* candidate : starts_near_end)
                {
                    if(processed_lines.find(candidate) != processed_lines.end())
                    {
                        continue; //Already processed this line before. It's linked to something else.
                    }
                    const coord_t dist2 = vSize2(candidate->junctions.front().p - chain_start);
                    if(dist2 < nearest_dist2)
                    {
                        nearest = candidate;
                        nearest_dist2 = dist2;
                        nearest_reverse = false;
                        nearest_before = false;
                    }
                }
                for(const ExtrusionLine* candidate : ends_near_end)
                {
                    if(processed_lines.find(candidate) != processed_lines.end())
                    {
                        continue;
                    }
                    const coord_t dist2 = vSize2(candidate->junctions.back().p - chain_start);
                    if(dist2 < nearest_dist2)
                    {
                        nearest = candidate;
                        nearest_dist2 = dist2;
                        nearest_reverse = true;
                        nearest_before = false;
                    }
                }
            }

            //Now serialize the entire chain into one polygon.
            output.emplace_back();
            for(size_t i = 0; i < chain.size(); ++i)
            {
                if(!is_reversed[i])
                {
                    for(const ExtrusionJunction& junction : chain[i]->junctions)
                    {
                        output.back().add(junction.p);
                    }
                }
                else
                {
                    for(auto junction = chain[i]->junctions.rbegin(); junction != chain[i]->junctions.rend(); ++junction)
                    {
                        output.back().add(junction->p);
                    }
                }
            }
        }
    }
}


std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> WallToolPaths::getWeakOrder(const VariableWidthPaths& input, const bool outer_to_inner, const bool include_transitive)
{
    size_t max_inset_idx = 0;
    Polygons all_polygons;
    std::unordered_map<size_t, const ExtrusionLine*> poly_idx_to_extrusionline;
    for (const VariableWidthLines& inset : input)
    {
        for (const ExtrusionLine& line : inset)
        {
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
    }

    std::vector<std::vector<size_t>> nesting = all_polygons.getNesting();

    {
        SVG svg("/tmp/nesting.svg", AABB(all_polygons));
        svg.writePolygons(all_polygons, SVG::Color::BLUE);
        for (size_t i = 0; i < all_polygons.size(); i++)
        {
            for (size_t child_idx : nesting[i])
            {
                svg.writeArrow(all_polygons[i][0], all_polygons[child_idx][1]);
            }
        }
    }

    std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> result;

    getWeakOrder(0, poly_idx_to_extrusionline, nesting, max_inset_idx, outer_to_inner, result);

    if (include_transitive)
    {
        std::unordered_multimap<const ExtrusionLine*, const ExtrusionLine*> order_mapping;
        for (auto [from, to] : result)
        {
            order_mapping.emplace(from, to);
        }
        std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> transitive_order = result;
        for (auto [from, to] : result)
        {
            std::queue<const ExtrusionLine*> starts_of_next_relation;
            starts_of_next_relation.emplace(to);
            while ( ! starts_of_next_relation.empty())
            {
                const ExtrusionLine* start_of_next_relation = starts_of_next_relation.front();
                starts_of_next_relation.pop();
                auto range = order_mapping.equal_range(start_of_next_relation);
                for (auto it = range.first; it != range.second; ++it)
                {
                    auto [ next_from, next_to ] = *it;
                    starts_of_next_relation.emplace(next_to);
                    transitive_order.emplace(from, next_to);
                }
            }
        }
        result = transitive_order;
    }

    return result;
}

void WallToolPaths::getWeakOrder(size_t node_idx, const std::unordered_map<size_t, const ExtrusionLine*>& poly_idx_to_extrusionline, const std::vector<std::vector<size_t>>& nesting, size_t max_inset_idx, const bool outer_to_inner, std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>>& result)
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
        {
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
                if (other_child->is_odd)
                {
                    assert(other_child->inset_idx == child->inset_idx + 1);
                    result.emplace(child, other_child);
                }
                else
                {
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
        {
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

} // namespace cura
