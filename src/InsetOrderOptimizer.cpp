// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "InsetOrderOptimizer.h"

#include <functional>
#include <tuple>

#include <range/v3/algorithm/max.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/addressof.hpp>
#include <range/v3/view/any_view.hpp>
#include <range/v3/view/drop.hpp>
#include <range/v3/view/drop_last.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/remove_if.hpp>
#include <range/v3/view/reverse.hpp>
#include <range/v3/view/take_exactly.hpp>
#include <range/v3/view/transform.hpp>

#include "ExtruderTrain.h"
#include "FffGcodeWriter.h"
#include "LayerPlan.h"
#include "utils/views/convert.h"
#include "utils/views/dfs.h"

namespace rg = ranges;
namespace rv = ranges::views;

namespace cura
{

InsetOrderOptimizer::InsetOrderOptimizer(
    const FffGcodeWriter& gcode_writer,
    const SliceDataStorage& storage,
    LayerPlan& gcode_layer,
    const Settings& settings,
    const int extruder_nr,
    const GCodePathConfig& inset_0_default_config,
    const GCodePathConfig& inset_X_default_config,
    const GCodePathConfig& inset_0_roofing_config,
    const GCodePathConfig& inset_X_roofing_config,
    const GCodePathConfig& inset_0_flooring_config,
    const GCodePathConfig& inset_X_flooring_config,
    const GCodePathConfig& inset_0_bridge_config,
    const GCodePathConfig& inset_X_bridge_config,
    const bool retract_before_outer_wall,
    const coord_t wall_0_wipe_dist,
    const coord_t wall_x_wipe_dist,
    const size_t wall_0_extruder_nr,
    const size_t wall_x_extruder_nr,
    const ZSeamConfig& z_seam_config,
    const std::vector<VariableWidthLines>& paths,
    const Point2LL& model_center_point,
    const Shape& disallowed_areas_for_seams,
    const bool scarf_seam,
    const bool smooth_speed,
    const Shape& overhang_areas)
    : gcode_writer_(gcode_writer)
    , storage_(storage)
    , gcode_layer_(gcode_layer)
    , settings_(settings)
    , extruder_nr_(extruder_nr)
    , inset_0_default_config_(inset_0_default_config)
    , inset_X_default_config_(inset_X_default_config)
    , inset_0_roofing_config_(inset_0_roofing_config)
    , inset_X_roofing_config_(inset_X_roofing_config)
    , inset_0_flooring_config_(inset_0_flooring_config)
    , inset_X_flooring_config_(inset_X_flooring_config)
    , inset_0_bridge_config_(inset_0_bridge_config)
    , inset_X_bridge_config_(inset_X_bridge_config)
    , retract_before_outer_wall_(retract_before_outer_wall)
    , wall_0_wipe_dist_(wall_0_wipe_dist)
    , wall_x_wipe_dist_(wall_x_wipe_dist)
    , wall_0_extruder_nr_(wall_0_extruder_nr)
    , wall_x_extruder_nr_(wall_x_extruder_nr)
    , z_seam_config_(z_seam_config)
    , paths_(paths)
    , layer_nr_(gcode_layer.getLayerNr())
    , model_center_point_(model_center_point)
    , disallowed_areas_for_seams_{ disallowed_areas_for_seams }
    , scarf_seam_(scarf_seam)
    , smooth_speed_(smooth_speed)
    , overhang_areas_(overhang_areas)
{
}

bool InsetOrderOptimizer::addToLayer()
{
    // Settings & configs:
    const auto pack_by_inset = ! settings_.get<bool>("optimize_wall_printing_order");
    const auto inset_direction = settings_.get<InsetDirection>("inset_direction");
    const auto alternate_walls = settings_.get<bool>("material_alternate_walls");

    const bool outer_to_inner = inset_direction == InsetDirection::OUTSIDE_IN;
    const bool use_one_extruder = wall_0_extruder_nr_ == wall_x_extruder_nr_;
    const bool current_extruder_is_wall_x = wall_x_extruder_nr_ == extruder_nr_;

    const bool reverse = shouldReversePath(use_one_extruder, current_extruder_is_wall_x, outer_to_inner);
    const bool use_shortest_for_inner_walls = outer_to_inner;
    auto walls_to_be_added = getWallsToBeAdded(reverse, use_one_extruder);

    const auto order = pack_by_inset ? getInsetOrder(walls_to_be_added, outer_to_inner) : getRegionOrder(walls_to_be_added, outer_to_inner);

    constexpr Ratio flow = 1.0_r;

    bool added_something = false;

    constexpr bool detect_loops = false;
    constexpr Shape* combing_boundary = nullptr;
    const auto group_outer_walls = settings_.get<bool>("group_outer_walls");
    // When we alternate walls, also alternate the direction at which the first wall starts in.
    // On even layers we start with normal direction, on odd layers with inverted direction.
    PathOrderOptimizer<const ExtrusionLine*> order_optimizer(
        gcode_layer_.getLastPlannedPositionOrStartingPosition(),
        z_seam_config_,
        detect_loops,
        combing_boundary,
        reverse,
        order,
        group_outer_walls,
        disallowed_areas_for_seams_,
        use_shortest_for_inner_walls,
        overhang_areas_);

    for (auto& line : walls_to_be_added)
    {
        if (line.is_closed_)
        {
            std::optional<size_t> force_start;
            if (! settings_.get<bool>("z_seam_on_vertex"))
            {
                // If the user indicated that we may deviate from the vertices for the seam, we can insert a seam point, if needed.
                force_start = insertSeamPoint(line);
            }
            order_optimizer.addPolygon(&line, force_start, line.is_outer_wall());
        }
        else
        {
            order_optimizer.addPolyline(&line);
        }
    }

    order_optimizer.optimize();

    for (const PathOrdering<const ExtrusionLine*>& path : order_optimizer.paths_)
    {
        if (path.vertices_->empty())
        {
            continue;
        }

        const bool is_outer_wall = path.vertices_->inset_idx_ == 0; // or thin wall 'gap filler'
        const bool is_gap_filler = path.vertices_->is_odd_;
        const GCodePathConfig& default_config = is_outer_wall ? inset_0_default_config_ : inset_X_default_config_;
        const GCodePathConfig& roofing_config = is_outer_wall ? inset_0_roofing_config_ : inset_X_roofing_config_;
        const GCodePathConfig& flooring_config = is_outer_wall ? inset_0_flooring_config_ : inset_X_flooring_config_;
        const GCodePathConfig& bridge_config = is_outer_wall ? inset_0_bridge_config_ : inset_X_bridge_config_;
        const coord_t wipe_dist = is_outer_wall && ! is_gap_filler ? wall_0_wipe_dist_ : wall_x_wipe_dist_;
        const bool retract_before = is_outer_wall ? retract_before_outer_wall_ : false;
        const bool scarf_seam = scarf_seam_ && is_outer_wall;
        const bool smooth_speed = smooth_speed_ && is_outer_wall;

        const bool revert_inset = alternate_walls && (path.vertices_->inset_idx_ % 2 != 0);
        const bool revert_layer = alternate_walls && (layer_nr_ % 2 != 0);
        const bool backwards = path.backwards_ != (revert_inset != revert_layer);
        const size_t start_index = (backwards != path.backwards_) ? path.vertices_->size() - (path.start_vertex_ + 1) : path.start_vertex_;
        const bool linked_path = ! path.is_closed_;

        gcode_layer_.setIsInside(true); // Going to print walls, which are always inside.
        gcode_layer_.addWall(
            *path.vertices_,
            start_index,
            settings_,
            default_config,
            roofing_config,
            flooring_config,
            bridge_config,
            wipe_dist,
            flow,
            retract_before,
            path.is_closed_,
            backwards,
            linked_path,
            scarf_seam,
            smooth_speed);
        added_something = true;
    }
    return added_something;
}

std::optional<size_t> InsetOrderOptimizer::insertSeamPoint(ExtrusionLine& closed_line)
{
    assert(closed_line.is_closed_);
    assert(closed_line.size() >= 3);

    Point2LL request_point;
    switch (z_seam_config_.type_)
    {
    case EZSeamType::USER_SPECIFIED:
        request_point = z_seam_config_.pos_;
        break;
    case EZSeamType::SHORTEST:
        request_point = gcode_layer_.getLastPlannedPositionOrStartingPosition();
        break;
    default:
        return std::nullopt;
    }

    // Find the 'closest' point on the polygon to the request_point.
    Point2LL closest_point;
    size_t closest_junction_idx = 0;
    coord_t closest_distance_sqd = std::numeric_limits<coord_t>::max();
    bool should_recalculate_closest = false;
    if (z_seam_config_.type_ == EZSeamType::USER_SPECIFIED)
    {
        // For user-defined seams you usually don't _actually_ want the _closest_ point, per-se,
        // since you want the seam-line to be continuous in 3D space.
        // To that end, take the center of the 3D model (not of the current polygon, as that would give the same problems)
        // and project the point along the ray from the center to the request_point.

        const Point2LL ray_origin = model_center_point_;
        request_point = ray_origin + (request_point - ray_origin) * 10;

        for (const auto& [i, junction] : closed_line.junctions_ | ranges::views::enumerate)
        {
            // NOTE: Maybe rewrite this once we can use C++23 ranges::views::adjacent
            const auto& next_junction = closed_line.junctions_[(i + 1) % closed_line.junctions_.size()];

            float t, u;
            if (LinearAlg2D::segmentSegmentIntersection(ray_origin, request_point, junction.p_, next_junction.p_, t, u))
            {
                const Point2LL intersection = ray_origin + (request_point - ray_origin) * t;
                const coord_t distance_sqd = vSize2(request_point - intersection);
                if (distance_sqd < closest_distance_sqd)
                {
                    closest_point = intersection;
                    closest_distance_sqd = distance_sqd;
                    closest_junction_idx = i;
                }
            }
        }
    }
    if (closest_distance_sqd >= std::numeric_limits<coord_t>::max())
    {
        // If it the method isn't 'user-defined', or the attempt to do user-defined above failed
        // (since we don't take the center of the polygon, but of the model, there's a chance there's no intersection),
        // then just find the closest point on the polygon.

        for (const auto& [i, junction] : closed_line.junctions_ | ranges::views::enumerate)
        {
            const auto& next_junction = closed_line.junctions_[(i + 1) % closed_line.junctions_.size()];
            const coord_t distance_sqd = LinearAlg2D::getDist2FromLineSegment(junction.p_, request_point, next_junction.p_);
            if (distance_sqd < closest_distance_sqd)
            {
                closest_distance_sqd = distance_sqd;
                closest_junction_idx = i;
            }
        }
        should_recalculate_closest = true;
    }

    const auto& start_pt = closed_line.junctions_[closest_junction_idx];
    const auto& end_pt = closed_line.junctions_[(closest_junction_idx + 1) % closed_line.junctions_.size()];
    if (should_recalculate_closest)
    {
        // In the second case (see above) the closest point hasn't actually been calculated yet,
        // since in that case we'de need the start and end points. So do that here.
        closest_point = LinearAlg2D::getClosestOnLineSegment(request_point, start_pt.p_, end_pt.p_);
    }
    constexpr coord_t smallest_dist_sqd = 25;
    if (vSize2(closest_point - start_pt.p_) <= smallest_dist_sqd)
    {
        // If the closest point is very close to the start point, just use it instead.
        return closest_junction_idx;
    }
    if (vSize2(closest_point - end_pt.p_) <= smallest_dist_sqd)
    {
        // If the closest point is very close to the end point, just use it instead.
        return (closest_junction_idx + 1) % closed_line.junctions_.size();
    }

    // NOTE: This could also be done on a single axis (skipping the implied sqrt), but figuring out which one and then using the right values became a bit messy/verbose.
    const coord_t total_dist = vSize(end_pt.p_ - start_pt.p_);
    const coord_t start_dist = vSize(closest_point - start_pt.p_);
    const coord_t end_dist = vSize(closest_point - end_pt.p_);
    const coord_t w = ((end_pt.w_ * end_dist) / total_dist) + ((start_pt.w_ * start_dist) / total_dist);

    closed_line.junctions_.insert(closed_line.junctions_.begin() + closest_junction_idx + 1, ExtrusionJunction(closest_point, w, start_pt.perimeter_index_));
    return closest_junction_idx + 1;
}

InsetOrderOptimizer::value_type InsetOrderOptimizer::getRegionOrder(const std::vector<ExtrusionLine>& extrusion_lines, const bool outer_to_inner)
{
    if (extrusion_lines.empty())
    {
        // Early out
        return {};
    }

    // view on the extrusion lines, sorted by area
    const std::vector<const ExtrusionLine*> sorted_extrusion_lines = [&extrusion_lines]()
    {
        auto extrusion_lines_area = extrusion_lines | ranges::views::addressof
                                  | ranges::views::transform(
                                        [](const ExtrusionLine* line)
                                        {
                                            const Polygon poly = line->toPolygon();
                                            AABB aabb;
                                            aabb.include(poly);
                                            return std::make_pair(line, aabb.area());
                                        })
                                  | ranges::to_vector;

        ranges::sort(
            extrusion_lines_area,
            [](const auto& lhs, const auto& rhs)
            {
                return std::get<1>(lhs) < std::get<1>(rhs);
            });

        return extrusion_lines_area
             | ranges::views::transform(
                   [](const auto& pair)
                   {
                       return std::get<0>(pair);
                   })
             | ranges::to_vector;
    }();

    // graph will contain the parent-child relationships between the extrusion lines
    // an edge is added for both the parent to child and child to parent relationship
    std::unordered_multimap<const ExtrusionLine*, const ExtrusionLine*> graph;
    // during the loop we maintain a list of invariant parents; these are the parents
    // that we have found so far
    std::unordered_set<const ExtrusionLine*> invariant_outer_parents;
    for (const auto& extrusion_line : sorted_extrusion_lines)
    {
        // Create a polygon representing the inner area of the extrusion line; any
        // point inside this polygon is considered to the child of the extrusion line.
        Shape hole_polygons;
        if (extrusion_line->is_closed_)
        {
            hole_polygons.push_back(extrusion_line->toPolygon());
        }

        if (hole_polygons.empty())
        {
            invariant_outer_parents.emplace(extrusion_line);
            continue;
        }

        // go through all the invariant parents and see if they are inside the hole polygon
        // if they are, then that means we have found a child for this extrusion line
        std::vector<const ExtrusionLine*> removed_parent_invariants;
        for (const ExtrusionLine* invariant_parent : invariant_outer_parents)
        {
            if (hole_polygons.inside(invariant_parent->junctions_[0].p_, false))
            {
                // The root polygon is inside the location polygon. It is no longer a root in the graph we are building.
                // Add this relationship (locator <-> root) to the graph, and remove root from roots.
                graph.emplace(extrusion_line, invariant_parent);
                graph.emplace(invariant_parent, extrusion_line);
                removed_parent_invariants.emplace_back(invariant_parent);
            }
        }
        for (const auto& node : removed_parent_invariants)
        {
            invariant_outer_parents.erase(node);
        }

        // the current extrusion line is now an invariant parent
        invariant_outer_parents.emplace(extrusion_line);
    }

    const std::vector<const ExtrusionLine*> outer_walls = extrusion_lines | ranges::views::filter(&ExtrusionLine::is_outer_wall) | ranges::views::addressof | ranges::to_vector;

    // find for each line the closest outer line, and store this in closest_outer_wall_line
    std::unordered_map<const ExtrusionLine*, const ExtrusionLine*> closest_outer_wall_line;
    std::unordered_map<const ExtrusionLine*, unsigned int> min_depth;
    for (const ExtrusionLine* outer_wall : outer_walls)
    {
        const std::function<void(const ExtrusionLine*, const unsigned int)> update_nodes
            = [&outer_wall, &min_depth, &closest_outer_wall_line](const ExtrusionLine* current_line, const unsigned int depth)
        {
            if (min_depth.find(current_line) == min_depth.end() || depth < min_depth[current_line])
            {
                min_depth[current_line] = depth;
                closest_outer_wall_line[current_line] = outer_wall;
            }
        };
        actions::dfs_depth_state(outer_wall, graph, update_nodes);
    }

    // for each of the outer walls, perform a dfs until we have found an extrusion line that is
    // _not_ closest to the current outer wall, then stop the dfs traversal for that branch. For
    // each extrusion $e$ traversed in the dfs, add an order constraint between to $e$ and the
    // previous line in the dfs traversal of $e$.
    std::unordered_multimap<const ExtrusionLine*, const ExtrusionLine*> order;
    for (const ExtrusionLine* outer_wall : outer_walls)
    {
        const std::function<void(const ExtrusionLine*, const ExtrusionLine*)> set_order_constraints
            = [&order, &closest_outer_wall_line, &outer_wall, &outer_to_inner](const auto& current_line, const auto& parent_line)
        {
            // if the closest
            if (closest_outer_wall_line[current_line] == outer_wall && parent_line != nullptr)
            {
                // flip the key values if we want to print from inner to outer walls
                if (outer_to_inner)
                {
                    order.insert(std::make_pair(parent_line, current_line));
                }
                else
                {
                    order.insert(std::make_pair(current_line, parent_line));
                }
            }
        };

        actions::dfs_parent_state(outer_wall, graph, set_order_constraints);
    }

    return order;
}

InsetOrderOptimizer::value_type InsetOrderOptimizer::getInsetOrder(const auto& input, const bool outer_to_inner)
{
    value_type order;

    std::vector<std::vector<const ExtrusionLine*>> walls_by_inset;
    std::vector<std::vector<const ExtrusionLine*>> fillers_by_inset;

    for (const auto& line : input)
    {
        if (line.is_odd_)
        {
            if (line.inset_idx_ >= fillers_by_inset.size())
            {
                fillers_by_inset.resize(line.inset_idx_ + 1);
            }
            fillers_by_inset[line.inset_idx_].emplace_back(&line);
        }
        else
        {
            if (line.inset_idx_ >= walls_by_inset.size())
            {
                walls_by_inset.resize(line.inset_idx_ + 1);
            }
            walls_by_inset[line.inset_idx_].emplace_back(&line);
        }
    }
    for (size_t inset_idx = 0; inset_idx + 1 < walls_by_inset.size(); inset_idx++)
    {
        for (const ExtrusionLine* line : walls_by_inset[inset_idx])
        {
            for (const ExtrusionLine* inner_line : walls_by_inset[inset_idx + 1])
            {
                const ExtrusionLine* before = inner_line;
                const ExtrusionLine* after = line;
                if (outer_to_inner)
                {
                    std::swap(before, after);
                }
                order.emplace(before, after);
            }
        }
    }
    for (size_t inset_idx = 1; inset_idx < fillers_by_inset.size(); inset_idx++)
    {
        for (const ExtrusionLine* line : fillers_by_inset[inset_idx])
        {
            if (inset_idx - 1 >= walls_by_inset.size())
                continue;
            for (const ExtrusionLine* enclosing_wall : walls_by_inset[inset_idx - 1])
            {
                order.emplace(enclosing_wall, line);
            }
        }
    }

    return order;
}

constexpr bool InsetOrderOptimizer::shouldReversePath(const bool use_one_extruder, const bool current_extruder_is_wall_x, const bool outer_to_inner)
{
    if (use_one_extruder && current_extruder_is_wall_x)
    {
        return ! outer_to_inner;
    }
    return current_extruder_is_wall_x;
}

std::vector<ExtrusionLine> InsetOrderOptimizer::getWallsToBeAdded(const bool reverse, const bool use_one_extruder)
{
    if (paths_.empty())
    {
        return {};
    }
    rg::any_view<VariableWidthLines> view;
    if (reverse)
    {
        if (use_one_extruder)
        {
            view = paths_ | rv::reverse;
        }
        else
        {
            view = paths_ | rv::reverse | rv::drop_last(1);
        }
    }
    else
    {
        if (use_one_extruder)
        {
            view = paths_ | rv::all;
        }
        else
        {
            view = paths_ | rv::take_exactly(1);
        }
    }
    return view | rv::join | rv::remove_if(rg::empty) | rg::to_vector;
}
} // namespace cura
