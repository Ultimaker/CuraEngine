// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHORDEROPTIMIZER_H
#define PATHORDEROPTIMIZER_H

#include <numbers>
#include <unordered_set>

#include <range/v3/algorithm/partition_copy.hpp>
#include <range/v3/iterator/insert_iterators.hpp>
#include <range/v3/view/addressof.hpp>
#include <range/v3/view/drop_last.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/reverse.hpp>
#include <spdlog/spdlog.h>

#include "pathPlanning/CombPath.h" //To calculate the combing distance if we want to use combing.
#include "pathPlanning/LinePolygonsCrossings.h" //To prevent calculating combing distances if we don't cross the combing borders.
#include "path_ordering.h"
#include "settings/EnumSettings.h" //To get the seam settings.
#include "settings/ZSeamConfig.h" //To read the seam configuration.
#include "utils/linearAlg2D.h" //To find the angle of corners to hide seams.
#include "utils/math.h"
#include "utils/polygonUtils.h"
#include "utils/scoring/BestElementFinder.h"
#include "utils/scoring/CornerScoringCriterion.h"
#include "utils/scoring/DistanceScoringCriterion.h"
#include "utils/scoring/ExclusionAreaScoringCriterion.h"
#include "utils/scoring/RandomScoringCriterion.h"
#include "utils/views/dfs.h"

namespace cura
{

/*!
 * Path order optimization class.
 *
 * Utility class for optimizing the order in which things are printed, by
 * minimizing the distance traveled between different items to be printed. For
 * each item to be printed, it also chooses a starting point as to where on the
 * polygon or polyline to start printing, and determines which direction to
 * print in.
 *
 * To use this class, first create an instance and provide some parameters as
 * metadata. Then add polygons and polylines to the class. Then call the
 * \ref optimize function to compute the optimization. Finally, print the
 * polygons and polylines in the \ref paths field in the order in which they are
 * given.
 *
 * In the output of this class, polylines and polygons are combined into a
 * single vector: \ref paths . Each path contains a pointer to the original
 * polygon data, as well as whether that data represented a polygon or a
 * polyline, which direction to print that path in, and where to start along the
 * path.
 *
 * The optimizer will always start a polyline from either end, never halfway.
 * The Z seam is not used for these.
 * \tparam PathType The type of paths that will be optimized by this optimizer.
 * Make sure that a specialization is available for \ref get_vertex and
 * \ref get_size in order for the optimizer to know how to read information from
 * your path.
 */
template<typename Path>
class PathOrderOptimizer
{
public:
    using OrderablePath = PathOrdering<Path>;
    /* Areas defined here are not allowed to have the start the prints */
    Shape disallowed_area_for_seams;
    /*!
     * After optimizing, this contains the paths that need to be printed in the
     * correct order.
     *
     * Each path contains the information necessary to print the parts: A
     * pointer to the vertex data, whether or not to close the loop, the
     * direction in which to print the path and where to start the path.
     */
    std::vector<OrderablePath> paths_;

    /*!
     * Maps the path implementation to it's orderable path container
     */
    std::unordered_map<Path, OrderablePath*> vertices_to_paths_;

    /*!
     * The location where the nozzle is assumed to start from before printing
     * these parts.
     */
    Point2LL start_point_;

    /*!
     * Seam settings.
     */
    ZSeamConfig seam_config_;

    static const std::unordered_multimap<Path, Path> no_order_requirements_;

    /*!
     * Construct a new optimizer.
     *
     * This doesn't actually optimize the order yet, so the ``paths`` field will
     * not be filled yet.
     * \param start_point The location where the nozzle is assumed to start from
     * before printing these parts.
     * \param config Seam settings.
     * \param detect_loops Whether to try to close polylines if the endpoints
     * are close together. If they are just a few microns apart, it will merge
     * the two endpoints together and pretends that the loop is closed, turning
     * it into a polygon.
     * \param combing_boundary Boundary to avoid when making travel moves.
     */
    PathOrderOptimizer(
        const Point2LL& start_point,
        const ZSeamConfig seam_config = ZSeamConfig(),
        const bool detect_loops = false,
        const Shape* combing_boundary = nullptr,
        const bool reverse_direction = false,
        const std::unordered_multimap<Path, Path>& order_requirements = no_order_requirements_,
        const bool group_outer_walls = false,
        const Shape& disallowed_areas_for_seams = {},
        const bool use_shortest_for_inner_walls = false,
        const Shape& overhang_areas = Shape())
        : start_point_(start_point)
        , seam_config_(seam_config)
        , combing_boundary_((combing_boundary != nullptr && ! combing_boundary->empty()) ? combing_boundary : nullptr)
        , detect_loops_(detect_loops)
        , reverse_direction_(reverse_direction)
        , _group_outer_walls(group_outer_walls)
        , order_requirements_(&order_requirements)
        , disallowed_area_for_seams{ disallowed_areas_for_seams }
        , use_shortest_for_inner_walls_(use_shortest_for_inner_walls)
        , overhang_areas_(overhang_areas)
    {
    }

    /*!
     * Add a new polygon to be optimized.
     * \param polygon The polygon to optimize.
     */
    void addPolygon(const Path& polygon, std::optional<size_t> force_start_index = std::nullopt, const bool is_outer_wall = false)
    {
        constexpr bool is_closed = true;
        paths_.emplace_back(polygon, is_closed);
        paths_.back().force_start_index_ = force_start_index;
        paths_.back().is_outer_wall = is_outer_wall;
    }

    /*!
     * Add a new polyline to be optimized.
     * \param polyline The polyline to optimize.
     */
    void addPolyline(const Path& polyline)
    {
        constexpr bool is_closed = false;
        paths_.emplace_back(polyline, is_closed);
    }

    /*!
     * Perform the calculations to optimize the order of the parts.
     *
     * This reorders the \ref paths field and fills their starting vertices and
     * directions.
     */
    void optimize(bool precompute_start = true)
    {
        if (paths_.empty())
        {
            return;
        }

        // Get the vertex data and store it in the paths.
        for (auto& path : paths_)
        {
            path.converted_ = &path.getVertexData();
            vertices_to_paths_.emplace(path.vertices_, &path);
        }

        // If necessary, check polylines to see if they are actually polygons.
        if (detect_loops_)
        {
            for (auto& path : paths_)
            {
                if (! path.is_closed_)
                {
                    // If we want to detect chains, first check if some of the polylines are secretly polygons.
                    path.is_closed_ = isLoopingPolyline(path); // If it is, we'll set the seam position correctly later.
                }
            }
        }

        // Set actual used start point calculation strategy for each path
        for (auto& path : paths_)
        {
            if (use_shortest_for_inner_walls_ && ! path.is_outer_wall)
            {
                path.seam_config_ = ZSeamConfig(EZSeamType::SHORTEST);
                path.force_start_index_ = std::nullopt;
            }
            else
            {
                path.seam_config_ = seam_config_;
            }
        }

        // Add all vertices to a bucket grid so that we can find nearby endpoints quickly.
        const coord_t snap_radius = 10_mu; // 0.01mm grid cells. Chaining only needs to consider polylines which are next to each other.
        SparsePointGridInclusive<size_t> line_bucket_grid(snap_radius);
        for (const auto& [i, path] : paths_ | ranges::views::enumerate)
        {
            if (path.converted_->empty())
            {
                continue;
            }
            if (path.is_closed_)
            {
                for (const Point2LL& point : *path.converted_)
                {
                    line_bucket_grid.insert(point, i); // Store by index so that we can also mark them down in the `picked` vector.
                }
            }
            else // For polylines, only insert the endpoints. Those are the only places we can start from so the only relevant vertices to be near to.
            {
                line_bucket_grid.insert(path.converted_->front(), i);
                line_bucket_grid.insert(path.converted_->back(), i);
            }
        }

        // For some Z seam types the start position can be pre-computed.
        // This is faster since we don't need to re-compute the start position at each step then.
        if (precompute_start)
        {
            for (auto& path : paths_)
            {
                if (path.seam_config_.type_ == EZSeamType::RANDOM || path.seam_config_.type_ == EZSeamType::USER_SPECIFIED
                    || path.seam_config_.type_ == EZSeamType::SHARPEST_CORNER)
                {
                    if (! path.is_closed_ || path.converted_->empty())
                    {
                        continue; // Can't pre-compute the seam for open polylines since they're at the endpoint nearest to the current position.
                    }
                    path.start_vertex_ = findStartLocation(path, path.seam_config_.pos_);
                }
            }
        }

        std::vector<OrderablePath> optimized_order; // To store our result in. At the end we'll std::swap.

        if (order_requirements_->empty())
        {
            optimized_order = getOptimizedOrder(line_bucket_grid, snap_radius);
        }
        else
        {
            optimized_order = getOptimizerOrderWithConstraints(*order_requirements_);
        }


        if (reverse_direction_ && order_requirements_->empty())
        {
            std::vector<OrderablePath> reversed = reverseOrderPaths(optimized_order); // Reverse-insert the optimized order, to invert the ordering.
            std::swap(reversed, paths_);
        }
        else
        {
            std::swap(optimized_order, paths_);
        }

        combing_grid_.reset();
    }

protected:
    /*!
     * If \ref detect_loops is enabled, endpoints of polylines that are closer
     * than this distance together will be considered to be coincident, closing
     * that polyline into a polygon.
     */
    constexpr static coord_t _coincident_point_distance = 10;

    /*!
     * Bucket grid to store the locations of the combing boundary.
     *
     * This is cached in order to speed up the collision checking with the
     * combing boundary. We only need to generate this mapping once for the
     * combing boundary, since the combing boundary can't change.
     */
    std::unique_ptr<LocToLineGrid> combing_grid_;

    /*!
     * Boundary to avoid when making travel moves.
     */
    const Shape* combing_boundary_;

    /*!
     * Whether to check polylines to see if they are closed, before optimizing.
     *
     * If this is enabled, the optimizer will first attempt to find endpoints of
     * polylines that are very close together. If they are closer than
     * \ref coincident_point_distance, the polylines will be closed and form
     * polygons. This then allows the optimizer to decide on a seam location
     * that is not one of the endpoints of the polyline.
     */
    bool detect_loops_;

    /*!
     * Whether to reverse the ordering completely.
     *
     * This reverses the order in which parts are printed, and inverts the
     * direction of each path as well.
     */
    bool reverse_direction_;

    /*
     * Whether to print all outer walls in a group, one after another.
     *
     * If this is enabled outer walls will be printed first and then all other
     * walls will be printed. If reversed they will be printed last.
     */
    bool _group_outer_walls;

    /*!
     * Order requirements on the paths.
     * For each pair the first needs to be printe before the second.
     */
    const std::unordered_multimap<Path, Path>* order_requirements_;

    /*!
     * If true, we will compute the seam position of inner walls using a "shortest" seam configs, for inner walls that
     * are directly following an outer wall.
     */
    const bool use_shortest_for_inner_walls_;

    /*!
     * Contains the overhang areas, where we would prefer not to place the start locations of walls
     */
    const Shape overhang_areas_;

    std::vector<OrderablePath> getOptimizedOrder(SparsePointGridInclusive<size_t> line_bucket_grid, size_t snap_radius)
    {
        std::vector<OrderablePath> optimized_order; // To store our result in.

        Point2LL current_position = start_point_;

        std::unordered_map<OrderablePath*, bool> picked(paths_.size()); // Fixed size boolean flag for whether each path is already in the optimized vector.

        auto notPicked = [&picked](OrderablePath* c)
        {
            return ! picked[c];
        };

        while (optimized_order.size() < paths_.size())
        {
            // Use bucket grid to find paths within snap_radius
            std::vector<OrderablePath*> nearby_candidates;
            for (const auto i : line_bucket_grid.getNearbyVals(current_position, snap_radius))
            {
                nearby_candidates.push_back(&paths_[i]); // Convert bucket indexes to corresponding paths
            }

            std::vector<OrderablePath*> available_candidates;
            available_candidates.reserve(nearby_candidates.size());
            for (auto candidate : nearby_candidates | ranges::views::filter(notPicked))
            {
                available_candidates.push_back(candidate);
            }

            if (available_candidates.empty()) // We need to broaden our search through all candidates
            {
                for (auto path : paths_ | ranges::views::addressof | ranges::views::filter(notPicked))
                {
                    available_candidates.push_back(path);
                }
            }

            auto best_candidate = findClosestPath(current_position, available_candidates);

            auto best_path = best_candidate;
            optimized_order.push_back(*best_path);
            picked[best_path] = true;

            if (! best_path->converted_->empty()) // If all paths were empty, the best path is still empty. We don't upate the current position then.
            {
                if (best_path->is_closed_)
                {
                    current_position = (*best_path->converted_)[best_path->start_vertex_]; // We end where we started.
                }
                else
                {
                    // Pick the other end from where we started.
                    current_position = best_path->start_vertex_ == 0 ? best_path->converted_->back() : best_path->converted_->front();
                }
            }
        }

        return optimized_order;
    }

    std::vector<OrderablePath> getOptimizerOrderWithConstraints(const std::unordered_multimap<Path, Path>& order_requirements)
    {
        std::vector<OrderablePath> optimized_order; // To store our result in.

        // initialize the roots set with all possible nodes
        std::unordered_set<Path> roots;
        std::unordered_set<Path> leaves;
        std::unordered_map<Path, size_t> num_incoming_edges;
        for (const auto& path : paths_)
        {
            roots.insert(path.vertices_);
            leaves.insert(path.vertices_);
            num_incoming_edges.emplace(path.vertices_, 0);
        }

        // remove all edges from roots with an incoming edge
        // result is a set of nodes that have no incoming edges; these are by definition the roots
        for (const auto& [u, v] : order_requirements)
        {
            roots.erase(v);
            leaves.erase(u);
            num_incoming_edges.find(v)->second++;
        }

        // We used a shared visited set between runs of dfs. This is for the case when we reverse the ordering tree.
        // In this case two roots can share the same children nodes, but we don't want to print them twice.
        std::unordered_set<Path> visited;
        Point2LL current_position = start_point_;

        std::function<std::vector<Path>(const Path, const std::unordered_multimap<Path, Path>&)> get_neighbours
            = [&current_position, &num_incoming_edges, this](const Path current_node, const std::unordered_multimap<Path, Path>& graph)
        {
            std::vector<Path> order; // Output order to traverse neighbors

            const auto& [neighbour_begin, neighbour_end] = graph.equal_range(current_node);
            std::unordered_set<Path> candidates;
            for (const auto& [_, neighbour] : ranges::make_subrange(neighbour_begin, neighbour_end))
            {
                // we only want to visit nodes that have no incoming edges, this is for the situation where we
                // are printing paths from inner to outer. As the ordering tree is reversed, and we start traversing
                // from an arbitrary leaf we might encounter a junction. All paths from the other leaf-side(s) of the junction
                // should be printed before continuing the junctions. Only once every branch of the junction has been ordered
                // we can continue with the junction itself.
                if (num_incoming_edges.at(neighbour) == 0)
                {
                    candidates.insert(neighbour);
                }
            }

            auto local_current_position = current_position;
            while (! candidates.empty())
            {
                Path best_candidate = findClosestPathVertices(local_current_position, candidates);

                candidates.erase(best_candidate);
                order.push_back(best_candidate);

                // update local_current_position
                auto path = vertices_to_paths_[best_candidate];

                if (path->is_closed_)
                {
                    local_current_position = (*path->converted_)[path->start_vertex_]; // We end where we started.
                }
                else
                {
                    // Pick the other end from where we started.
                    local_current_position = path->start_vertex_ == 0 ? path->converted_->back() : path->converted_->front();
                }
            }

            return order;
        };

        const std::function<std::nullptr_t(const Path, const std::nullptr_t)> handle_node
            = [&current_position, &optimized_order, &order_requirements, &num_incoming_edges, this](const Path current_node, [[maybe_unused]] const std::nullptr_t state)
        {
            // We should make map from node <-> path for this stuff
            for (const auto& path : paths_)
            {
                if (path.vertices_ == current_node)
                {
                    if (path.is_closed_)
                    {
                        current_position = (*path.converted_)[path.start_vertex_]; // We end where we started.
                    }
                    else
                    {
                        // Pick the other end from where we started.
                        current_position = path.start_vertex_ == 0 ? path.converted_->back() : path.converted_->front();
                    }

                    // Add to optimized order
                    optimized_order.push_back(path);

                    // update incoming edges of neighbours since this path is handled
                    const auto& [neighbour_begin, neighbour_end] = order_requirements.equal_range(path.vertices_);
                    for (const auto& [_, neighbour] : ranges::make_subrange(neighbour_begin, neighbour_end))
                    {
                        num_incoming_edges.find(neighbour)->second--;
                    }

                    break;
                }
            }

            return nullptr;
        };

        if (_group_outer_walls)
        {
            if (reverse_direction_)
            {
                // When the order is reverse the leaves of the order requirement are the outer walls
                std::unordered_set<Path> outer_walls = leaves;

                // We don't want to add the outerwalls until after we finish all inner walls. Adding them to visited stops dfs from visiting these nodes.
                visited.insert(outer_walls.begin(), outer_walls.end());

                // Add all inner walls
                while (! roots.empty())
                {
                    Path root = findClosestPathVertices(current_position, roots);
                    roots.erase(root);
                    actions::dfs(root, order_requirements, handle_node, visited, nullptr, get_neighbours);
                }

                // Add all outer walls
                while (! outer_walls.empty())
                {
                    Path wall = findClosestPathVertices(current_position, outer_walls);
                    outer_walls.erase(wall);
                    handle_node(wall, nullptr);
                }
            }
            else
            {
                // The roots are the outer walls when reverse is false.
                std::unordered_set<Path> outer_walls = roots;

                // Add all outer walls
                std::unordered_set<Path> root_neighbours;
                while (! outer_walls.empty())
                {
                    Path outer_wall = findClosestPathVertices(current_position, outer_walls);
                    outer_walls.erase(outer_wall);

                    handle_node(outer_wall, nullptr);
                    visited.insert(outer_wall);
                    for (auto path : get_neighbours(outer_wall, order_requirements))
                    {
                        root_neighbours.emplace(path);
                    }
                }

                // Add all inner walls
                while (! root_neighbours.empty())
                {
                    Path root_neighbour = findClosestPathVertices(current_position, root_neighbours);
                    root_neighbours.erase(root_neighbour);
                    actions::dfs(root_neighbour, order_requirements, handle_node, visited, nullptr, get_neighbours);
                }
            }
        }
        else
        {
            while (! roots.empty())
            {
                Path root = findClosestPathVertices(current_position, roots);
                roots.erase(root);

                actions::dfs(root, order_requirements, handle_node, visited, nullptr, get_neighbours);
            }
        }

        return optimized_order;
    }

    std::vector<OrderablePath> reverseOrderPaths(std::vector<OrderablePath> pathsOrderPaths)
    {
        std::vector<OrderablePath> reversed;
        // Don't replace with swap, assign or insert. They require functions that we can't implement for all template arguments for Path.
        reversed.reserve(pathsOrderPaths.size());
        for (auto& path : pathsOrderPaths | ranges::views::reverse)
        {
            reversed.push_back(path);
            reversed.back().backwards_ = ! reversed.back().backwards_;
            if (! reversed.back().is_closed_)
            {
                reversed.back().start_vertex_ = reversed.back().converted_->size() - 1 - reversed.back().start_vertex_;
            }
        }

        return reversed;
    }

    Path findClosestPathVertices(Point2LL start_position, std::unordered_set<Path> candidate_paths)
    {
        std::vector<OrderablePath*> candidate_orderable_paths;

        for (auto& path : candidate_paths)
        {
            candidate_orderable_paths.push_back(vertices_to_paths_.at(path));
        }

        OrderablePath* best_candidate = findClosestPath(start_position, candidate_orderable_paths);
        return best_candidate->vertices_;
    }

    OrderablePath* findClosestPath(Point2LL start_position, std::vector<OrderablePath*> candidate_paths)
    {
        coord_t best_distance2 = std::numeric_limits<coord_t>::max();
        OrderablePath* best_candidate = 0;

        for (OrderablePath* path : candidate_paths)
        {
            if (path->converted_->empty()) // No vertices in the path. Can't find the start position then or really plan it in. Put that at the end.
            {
                if (best_distance2 == std::numeric_limits<coord_t>::max())
                {
                    best_candidate = path;
                }
                continue;
            }

            const bool precompute_start = path->seam_config_.type_ == EZSeamType::RANDOM || path->seam_config_.type_ == EZSeamType::USER_SPECIFIED
                                       || path->seam_config_.type_ == EZSeamType::SHARPEST_CORNER;
            if (! path->is_closed_ || ! precompute_start) // Find the start location unless we've already precomputed it.
            {
                path->start_vertex_ = findStartLocation(*path, start_position);
                if (! path->is_closed_) // Open polylines start at vertex 0 or vertex N-1. Indicate that they should be reversed if they start at N-1.
                {
                    path->backwards_ = path->start_vertex_ > 0;
                }
            }
            const Point2LL candidate_position = (*path->converted_)[path->start_vertex_];
            coord_t distance2 = getDirectDistance(start_position, candidate_position);
            if (distance2 < best_distance2
                && combing_boundary_) // If direct distance is longer than best combing distance, the combing distance can never be better, so only compute combing if necessary.
            {
                distance2 = getCombingDistance(start_position, candidate_position);
            }
            if (distance2 < best_distance2) // Closer than the best candidate so far.
            {
                best_candidate = path;
                best_distance2 = distance2;
            }
        }

        return best_candidate;
    }

    /**
     * @brief Analyze the positions in a path and determine the next optimal position based on a proximity criterion.
     *
     * This function iteratively examines positions along the given path, checking if the position is close to 3D model.
     * Each position is specified by an index, starting with `best_pos`. If the position is close to the model according to
     * `isVertexCloseToPolygonPath` function, the function recursively calls itself with the next position. This process is
     * repeated until all positions have been checked or `number_of_paths_analysed` becomes equal to `path_size`.
     * If `number_of_paths_analysed` becomes equal to `path_size`, it logs a warning and returns the current best position.
     *
     * @param best_pos The index of the initial position for analysis in the path.
     * @param path An OrderablePath instance containing the path to be examined.
     * @param number_of_paths_analysed Optionally, the initial index of paths analysed. Defaults to 0.
     * @return The index of the next optimal position in the path sequence. May be the same as the input `best_pos`,
     *         or may be incremented to a different location based on the proximity criterion.
     *
     * @note This function uses recursion to evaluate each position in the path.
     * @note The process stops prematurely if no start path is found for the support z seam distance.
     *       This typically happens when the distance of the support seam from the model is bigger than all the support wall points.
     */

    size_t pathIfZseamIsInDisallowedArea(size_t best_pos, const OrderablePath& path, size_t number_of_paths_analysed)
    {
        size_t path_size = path.converted_->size();
        if (path_size > number_of_paths_analysed)
        {
            if (! disallowed_area_for_seams.empty())
            {
                Point2LL current_candidate = (path.converted_)->at(best_pos);
                if (disallowed_area_for_seams.inside(current_candidate, true))
                {
                    size_t next_best_position = (path_size > best_pos + 1) ? best_pos + 1 : 0;
                    number_of_paths_analysed += 1;
                    best_pos = pathIfZseamIsInDisallowedArea(next_best_position, path, number_of_paths_analysed);
                }
            }
        }
        else
        {
            spdlog::warn("No start path found for support z seam distance");
            // We can also calculate the best point to start at this point.
            // This usually happens when the distance of support seam from model is bigger than the whole support wall points.
        }
        return best_pos;
    }


    /*!
     * Find the vertex which will be the starting point of printing a polygon or
     * polyline.
     *
     * This will be the seam location (for polygons) or the closest endpoint
     * (for polylines). Usually the seam location is some combination of being
     * the closest point and/or being a sharp inner or outer corner.
     * \param vertices The vertex data of a path. This will never be empty (so
     * no need to check again) but might have size 1.
     * \param target_pos The point that the starting vertex must be close to, if
     * applicable.
     * \param is_closed Whether the polygon is closed (a polygon) or not
     * (a polyline). If the path is not closed, it will choose between the two
     * endpoints rather than
     * \return An index to a vertex in that path where printing must start.
     */
    size_t findStartLocation(const OrderablePath& path, const Point2LL& target_pos)
    {
        if (! path.is_closed_)
        {
            // For polylines, the seam settings are not applicable. Simply choose the position closest to target_pos then.
            const coord_t back_distance
                = (combing_boundary_ == nullptr) ? getDirectDistance(path.converted_->back(), target_pos) : getCombingDistance(path.converted_->back(), target_pos);
            if (back_distance < getDirectDistance(path.converted_->front(), target_pos)
                || (combing_boundary_
                    && back_distance < getCombingDistance(path.converted_->front(), target_pos))) // Lazy or: Only compute combing distance if direct distance is closer.
            {
                return path.converted_->size() - 1; // Back end is closer.
            }
            return 0; // Front end is closer.
        }

        // Rest of the function only deals with (closed) polygons. We need to be able to find the seam location of those polygons.
        const PointsSet& points = *path.converted_;

        // ########## Step 1: define the main criteria to be applied and their weights
        // Standard weight for the "main" selection criterion, depending on the selected strategy. There should be
        // exactly one calculation using this criterion.
        BestElementFinder best_candidate_finder;
        BestElementFinder::CriteriaPass main_criteria_pass;
        main_criteria_pass.outsider_delta_threshold = 0.05;

        BestElementFinder::WeighedCriterion main_criterion;

        if (path.force_start_index_.has_value()) // Handles EZSeamType::USER_SPECIFIED with "seam_on_vertex" disabled
        {
            // Use a much smaller distance divider because we want points around the forced points to be filtered out very easily
            constexpr double distance_divider = 1.0;
            constexpr auto distance_type = DistanceScoringCriterion::DistanceType::Euclidian;
            main_criterion.criterion = std::make_shared<DistanceScoringCriterion>(points, points.at(path.force_start_index_.value()), distance_type, distance_divider);
        }
        else if (path.seam_config_.type_ == EZSeamType::SHORTEST || path.seam_config_.type_ == EZSeamType::USER_SPECIFIED)
        {
            main_criterion.criterion = std::make_shared<DistanceScoringCriterion>(points, target_pos);
        }
        else if (
            path.seam_config_.type_ == EZSeamType::SHARPEST_CORNER
            && (path.seam_config_.corner_pref_ != EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE && path.seam_config_.corner_pref_ != EZSeamCornerPrefType::PLUGIN))
        {
            main_criterion.criterion = std::make_shared<CornerScoringCriterion>(points, path.seam_config_.corner_pref_);
        }
        else if (path.seam_config_.type_ == EZSeamType::RANDOM)
        {
            main_criterion.criterion = std::make_shared<RandomScoringCriterion>();
        }

        if (main_criterion.criterion)
        {
            main_criteria_pass.criteria.push_back(main_criterion);
        }
        else
        {
            spdlog::warn("Missing main criterion calculator");
        }

        // Second criterion with heigher weight to avoid overhanging areas
        if (! overhang_areas_.empty())
        {
            BestElementFinder::WeighedCriterion overhang_criterion;
            overhang_criterion.weight = 2.0;
            overhang_criterion.criterion = std::make_shared<ExclusionAreaScoringCriterion>(points, overhang_areas_);
            main_criteria_pass.criteria.push_back(overhang_criterion);
        }

        best_candidate_finder.appendCriteriaPass(main_criteria_pass);

        // ########## Step 2: add fallback passes for criteria with very similar scores (e.g. corner on a cylinder)
        if (path.seam_config_.type_ == EZSeamType::SHARPEST_CORNER)
        {
            const AABB path_bounding_box(points);

            { // First fallback strategy is to take points on the back-most position
                auto fallback_criterion = std::make_shared<DistanceScoringCriterion>(points, path_bounding_box.max_, DistanceScoringCriterion::DistanceType::YOnly);
                constexpr double outsider_delta_threshold = 0.01;
                best_candidate_finder.appendSingleCriterionPass(fallback_criterion, outsider_delta_threshold);
            }

            { // Second fallback strategy, in case we still have multiple points that are aligned on Y (e.g. cube), take the right-most point
                auto fallback_criterion = std::make_shared<DistanceScoringCriterion>(points, path_bounding_box.max_, DistanceScoringCriterion::DistanceType::XOnly);
                best_candidate_finder.appendSingleCriterionPass(fallback_criterion);
            }
        }

        // ########## Step 3: apply the criteria to find the vertex with the best global score
        std::optional<size_t> best_i = best_candidate_finder.findBestElement(points.size());

        if (! disallowed_area_for_seams.empty())
        {
            best_i = pathIfZseamIsInDisallowedArea(best_i.value_or(0), path, 0);
        }
        return best_i.value_or(0);
    }

    /*!
     * Calculate the direct Euclidean distance to move from one point to
     * another.
     * \param a One point, to compute distance to \ref b.
     * \param b Another point, to compute distance to \ref a.
     * \return The distance between the two points.
     */
    coord_t getDirectDistance(const Point2LL& a, const Point2LL& b) const
    {
        return vSize2(a - b);
    }

    /*!
     * Calculate the distance that one would have to travel to move from A to B
     * while avoiding collisions with the combing boundary.
     *
     * This method assumes that there is a combing boundary. So
     * \ref combing_boundary should not be ``nullptr``.
     * \param a One point, to compute distance to \ref b.
     * \param b Another point, to compute distance to \ref a.
     * \return The combing distance between the two points.
     */
    coord_t getCombingDistance(const Point2LL& a, const Point2LL& b)
    {
        if (! PolygonUtils::polygonCollidesWithLineSegment(*combing_boundary_, a, b))
        {
            return getDirectDistance(a, b); // No collision with any line. Just compute the direct distance then.
        }
        if (paths_.size() > 100)
        {
            /* If we have many paths to optimize the order for, this combing
            calculation can become very expensive. Instead, penalize travels
            that hit the combing boundary with a static factor.*/
            return getDirectDistance(a, b) * 5;
        }

        if (combing_grid_ == nullptr)
        {
            constexpr coord_t grid_size = 2000; // 2mm grid cells. Smaller will use more memory, but reduce chance of unnecessary collision checks.
            combing_grid_ = PolygonUtils::createLocToLineGrid(*combing_boundary_, grid_size);
        }

        CombPath comb_path; // Output variable.
        constexpr coord_t rounding_error = -25;
        constexpr coord_t tiny_travel_threshold = 0;
        constexpr bool fail_on_unavoidable_obstacles = false;
        LinePolygonsCrossings::comb(*combing_boundary_, *combing_grid_, a, b, comb_path, rounding_error, tiny_travel_threshold, fail_on_unavoidable_obstacles);

        coord_t sum = 0;
        Point2LL last_point = a;
        for (const Point2LL& point : comb_path)
        {
            sum += vSize(point - last_point);
            last_point = point;
        }
        return sum * sum; // Squared distance, for fair comparison with direct distance.
    }

    bool isLoopingPolyline(const OrderablePath& path)
    {
        if (path.converted_->empty())
        {
            return false;
        }
        return vSize2(path.converted_->back() - path.converted_->front()) < _coincident_point_distance * _coincident_point_distance;
    }
};

template<typename Path>
const std::unordered_multimap<Path, Path> PathOrderOptimizer<Path>::no_order_requirements_;

} // namespace cura

#endif // PATHORDEROPTIMIZER_H
