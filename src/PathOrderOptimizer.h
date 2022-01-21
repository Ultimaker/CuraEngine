//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PATHORDEROPTIMIZER_H
#define PATHORDEROPTIMIZER_H

#include "pathPlanning/CombPath.h" //To calculate the combing distance if we want to use combing.
#include "pathPlanning/LinePolygonsCrossings.h" //To prevent calculating combing distances if we don't cross the combing borders.
#include "settings/EnumSettings.h" //To get the seam settings.
#include "settings/ZSeamConfig.h" //To read the seam configuration.
#include "utils/polygonUtils.h"
#include "utils/linearAlg2D.h" //To find the angle of corners to hide seams.

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
template<typename PathType>
class PathOrderOptimizer
{
public:
    /*!
     * Represents a path which has been optimized, the output of the
     * optimization.
     *
     * This small data structure contains the vertex data of a path,  where to
     * start along the path and in which direction to print it, as well as
     * whether the path should be closed (in case of a polygon) or open (in case
     * of a polyline.
     *
     * After optimization is completed, the \ref paths vector will be filled
     * with optimized paths.
     */
    struct Path
    {
        /*!
         * Construct a new path.
         *
         * The \ref converted field is not initialized yet. This can only be
         * done after all of the input paths have been added, to prevent
         * invalidating the pointer.
         */
        Path(const PathType& vertices, const bool is_closed = false, const size_t start_vertex = 0, const bool backwards = false)
        : vertices(vertices)
        , start_vertex(start_vertex)
        , is_closed(is_closed)
        , backwards(backwards)
        {
        }

        /*!
         * The vertex data of the path.
         */
        PathType vertices;

        /*!
         * Vertex data, converted into a Polygon so that the optimizer knows
         * how to deal with this data.
         */
        ConstPolygonPointer converted;

        /*!
         * Which vertex along the path to start printing with.
         *
         * If this path represents a polyline, this will always be one of the
         * endpoints of the path; either 0 or ``vertices->size() - 1``.
         */
        size_t start_vertex;

        Point getStartLocation() const
        {
            return (*converted)[start_vertex];
        }
        
        Point getEndLocation() const
        {
            if (is_closed)
            {
                return getStartLocation();
            }
            if (start_vertex == 0)
            {
                return converted->back();
            }
            else
            {
                return converted->front();
            }
        }

        /*!
         * Whether the path should be closed at the ends or not.
         *
         * If this path should be closed, it represents a polygon. If it should
         * not be closed, it represents a polyline.
         */
        bool is_closed;

        /*!
         * Whether the path should be traversed in backwards direction.
         *
         * For a polyline it may be more efficient to print the path in
         * backwards direction, if the last vertex is closer than the first.
         */
        bool backwards;
    };

    /*!
     * After optimizing, this contains the paths that need to be printed in the
     * correct order.
     *
     * Each path contains the information necessary to print the parts: A
     * pointer to the vertex data, whether or not to close the loop, the
     * direction in which to print the path and where to start the path.
     */
    std::vector<Path> paths;

    /*!
     * The location where the nozzle is assumed to start from before printing
     * these parts.
     */
    Point start_point;

    /*!
     * Seam settings.
     */
    ZSeamConfig seam_config;

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
    PathOrderOptimizer(const Point start_point, const ZSeamConfig seam_config = ZSeamConfig(), const bool detect_loops = false, const Polygons* combing_boundary = nullptr, const bool reverse_direction = false, bool selection_optimization = true)
    : start_point(start_point)
    , seam_config(seam_config)
    , combing_boundary((combing_boundary != nullptr && !combing_boundary->empty()) ? combing_boundary : nullptr)
    , detect_loops(detect_loops)
    , reverse_direction(reverse_direction)
    , selection_optimization(selection_optimization)
    {
    }

    /*!
     * Add a new polygon to be optimized.
     * \param polygon The polygon to optimize.
     */
    void addPolygon(const PathType& polygon)
    {
        constexpr bool is_closed = true;
        paths.emplace_back(polygon, is_closed);
    }

    /*!
     * Add a new polyline to be optimized.
     * \param polyline The polyline to optimize.
     */
    void addPolyline(const PathType& polyline)
    {
        constexpr bool is_closed = false;
        paths.emplace_back(polyline, is_closed);
    }

    /*!
     * Perform the calculations to optimize the order of the parts.
     *
     * This reorders the \ref paths field and fills their starting vertices and
     * directions.
     */
    void optimize()
    {
        if(paths.empty())
        {
            return;
        }

        //Get the vertex data and store it in the paths.
        cached_vertices.reserve(paths.size()); //Prevent pointer invalidation.
        for(Path& path : paths)
        {
            path.converted = getVertexData(path.vertices);
        }

        //If necessary, check polylines to see if they are actually polygons.
        if(detect_loops)
        {
            for(Path& path : paths)
            {
                if(!path.is_closed)
                {
                    //If we want to detect chains, first check if some of the polylines are secretly polygons.
                    path.is_closed = isLoopingPolyline(path); //If it is, we'll set the seam position correctly later.
                }
            }
        }
        
        if (selection_optimization)
        {
            optimizeSelection();
        }
        else
        {
            optimizeInsertion();
        }
        
        combing_grid.reset();
    }
    void optimizeSelection()
    {
        //Add all vertices to a bucket grid so that we can find nearby endpoints quickly.
        const coord_t snap_radius = 10_mu; // 0.01mm grid cells. Chaining only needs to consider polylines which are next to each other.
        SparsePointGridInclusive<size_t> line_bucket_grid(snap_radius);
        for(size_t i = 0; i < paths.size(); ++i)
        {
            const Path& path = paths[i];
            if (path.converted->empty())
            {
                continue;
            }
            if(path.is_closed)
            {
                for(const Point& point : *path.converted)
                {
                    line_bucket_grid.insert(point, i); //Store by index so that we can also mark them down in the `picked` vector.
                }
            }
            else //For polylines, only insert the endpoints. Those are the only places we can start from so the only relevant vertices to be near to.
            {
                line_bucket_grid.insert(path.converted->front(), i);
                line_bucket_grid.insert(path.converted->back(), i);
            }
        }

        //For some Z seam types the start position can be pre-computed.
        //This is faster since we don't need to re-compute the start position at each step then.
        const bool precompute_start = seam_config.type == EZSeamType::RANDOM || seam_config.type == EZSeamType::USER_SPECIFIED || seam_config.type == EZSeamType::SHARPEST_CORNER;
        if(precompute_start)
        {
            for(Path& path : paths)
            {
                if(!path.is_closed)
                {
                    continue; //Can't pre-compute the seam for open polylines since they're at the endpoint nearest to the current position.
                }
                if(path.converted->empty())
                {
                    continue;
                }
                path.start_vertex = findStartLocation(path, seam_config.pos);
            }
        }

        std::vector<bool> picked(paths.size(), false); //Fixed size boolean flag for whether each path is already in the optimized vector.
        Point current_position = start_point;
        std::vector<Path> optimized_order; //To store our result in. At the end we'll std::swap.
        optimized_order.reserve(paths.size());
        while(optimized_order.size() < paths.size())
        {
            size_t best_candidate = 0;
            coord_t best_distance2 = std::numeric_limits<coord_t>::max();

            //First see if we already know about some nearby paths due to the line bucket grid.
            std::vector<size_t> nearby_candidates = line_bucket_grid.getNearbyVals(current_position, snap_radius);
            std::vector<size_t> available_candidates;
            available_candidates.reserve(nearby_candidates.size());
            for(const size_t candidate : nearby_candidates)
            {
                if(picked[candidate])
                {
                    continue; //Not a valid candidate.
                }
                available_candidates.push_back(candidate);
            }
            if(available_candidates.empty()) //We may need to broaden our search through all candidates then.
            {
                for(size_t candidate = 0; candidate < paths.size(); ++candidate)
                {
                    if(picked[candidate])
                    {
                        continue; //Not a valid candidate.
                    }
                    available_candidates.push_back(candidate);
                }
            }

            for(const size_t candidate_path_index : available_candidates)
            {
                Path& path = paths[candidate_path_index];
                if(path.converted->empty()) //No vertices in the path. Can't find the start position then or really plan it in. Put that at the end.
                {
                    if(best_distance2 == std::numeric_limits<coord_t>::max())
                    {
                        best_candidate = candidate_path_index;
                    }
                    continue;
                }

                if(!path.is_closed || !precompute_start) //Find the start location unless we've already precomputed it.
                {
                    path.start_vertex = findStartLocation(path, current_position);
                    if(!path.is_closed) //Open polylines start at vertex 0 or vertex N-1. Indicate that they should be reversed if they start at N-1.
                    {
                        path.backwards = path.start_vertex > 0;
                    }
                }
                const Point candidate_position = (*path.converted)[path.start_vertex];
                coord_t distance2 = getDirectDistance(current_position, candidate_position);
                if(distance2 < best_distance2 && combing_boundary) //If direct distance is longer than best combing distance, the combing distance can never be better, so only compute combing if necessary.
                {
                    distance2 = getCombingDistance(current_position, candidate_position);
                }
                if(distance2 < best_distance2) //Closer than the best candidate so far.
                {
                    best_candidate = candidate_path_index;
                    best_distance2 = distance2;
                }
            }

            Path& best_path = paths[best_candidate];
            optimized_order.push_back(best_path);
            picked[best_candidate] = true;

            if(!best_path.converted->empty()) //If all paths were empty, the best path is still empty. We don't upate the current position then.
            {
                if(best_path.is_closed)
                {
                    current_position = (*best_path.converted)[best_path.start_vertex]; //We end where we started.
                }
                else
                {
                    //Pick the other end from where we started.
                    current_position = best_path.start_vertex == 0 ? best_path.converted->back() : best_path.converted->front();
                }
            }
        }

        //Apply the optimized order to the output field. Reverse if ordered to reverse.
        if(reverse_direction)
        {
            //Reverse-insert the optimized order, to invert the ordering.
            std::vector<Path> reversed;
            //Don't replace with swap, assign or insert. They require functions that we can't implement for all template arguments for PathType.
            reversed.reserve(optimized_order.size());
            for(auto it = optimized_order.rbegin(); it != optimized_order.rend(); it++)
            {
                reversed.push_back(*it);
                reversed.back().backwards = !reversed.back().backwards;
                if(!reversed.back().is_closed)
                {
                    reversed.back().start_vertex = reversed.back().converted->size() - 1 - reversed.back().start_vertex;
                }
            }
            std::swap(reversed, paths);
        }
        else
        {
            std::swap(optimized_order, paths);
        }
    }
    
    void optimizeInsertion()
    {
        if (seam_config.type == EZSeamType::USER_SPECIFIED)
        {
            start_point = seam_config.pos; // WARNING: is this correct?!
        }
        
        for (Path& path : paths)
        {
            if (!path.is_closed)
            {
                continue; //Can't pre-compute the seam for open polylines since they're at the endpoint nearest to the current position.
            }
            if (path.converted->empty())
            {
                continue;
            }
            path.start_vertex = findStartLocation(path, seam_config.pos);
        }

        std::list<std::pair<coord_t, Path>> optimized_order; // Distance to and next location
        
        std::function<coord_t (Point, Point)> getDistance = 
            combing_boundary ?
              std::function<coord_t (Point, Point)>( [this](Point from, Point to) { return getCombingDistance(from, to); } )
            : std::function<coord_t (Point, Point)>( [this](Point from, Point to) { return getDirectDistance(from, to); } );
        
        Path& first_path = paths.front(); // arbitrarily select the first path to add
        coord_t distance = std::sqrt(getDistance(start_point, first_path.getStartLocation()));
        optimized_order.emplace_back(distance, first_path);
            
        for (size_t to_be_inserted_idx = 1; to_be_inserted_idx < paths.size(); to_be_inserted_idx++)
        {
            typename std::list<std::pair<coord_t, Path>>::iterator best_pos_it = optimized_order.end();
            coord_t best_detour_distance = std::numeric_limits<coord_t>::max();
            coord_t best_dist_before_to_here = 0;
            coord_t best_dist_here_to_after = 0;
            bool best_is_flipped = false;

            Path& to_be_inserted = paths[to_be_inserted_idx];
            Point start_here = to_be_inserted.getStartLocation();
            Point end_here = to_be_inserted.getEndLocation();

            // TODO: use grid

            for (auto pos_after_it = optimized_order.end(); ; --pos_after_it) // update is performed at the end of the for loop
            {
                auto pos_before_it = pos_after_it;
                pos_before_it--;
                Path* path_before = (pos_after_it == optimized_order.begin()) ? nullptr : &pos_before_it->second;
                Path* path_after = (pos_after_it == optimized_order.end()) ? nullptr : &pos_after_it->second;
                Point loc_before = path_before? path_before->getEndLocation() : start_point;
                coord_t current_distance = path_after? pos_after_it->first : 0;
                
                coord_t dist_before = std::sqrt(getDistance(loc_before, start_here));
                coord_t dist_after = path_after ? std::sqrt(getDistance(end_here, path_after->getStartLocation())) : 0;

                if ( ! to_be_inserted.is_closed)
                {
                    coord_t flipped_dist_before = std::sqrt(getDistance(loc_before, end_here));
                    coord_t flipped_dist_after = path_after ? std::sqrt(getDistance(start_here, path_after->getStartLocation())) : 0;
                    best_is_flipped = false;
                    if (flipped_dist_before + flipped_dist_after < dist_before + dist_after)
                    {
                        dist_before = flipped_dist_before;
                        dist_after = flipped_dist_after;
                        best_is_flipped = true;
                    }
                }
                
                coord_t detour_distance = dist_before + dist_after - current_distance;
                if (detour_distance < best_detour_distance) // Less of a detour than the best candidate so far.
                {
                    best_pos_it = pos_after_it;
                    best_detour_distance = detour_distance;
                    best_dist_before_to_here = dist_before;
                    best_dist_here_to_after = dist_after;
                }
                
                if (pos_after_it == optimized_order.begin())
                {
                    break;
                }
            }

            if (best_is_flipped)
            {
                assert( ! to_be_inserted.is_closed);
                to_be_inserted.start_vertex = (to_be_inserted.start_vertex != 0) * (to_be_inserted.converted->size() - 1);
            }
            if (best_pos_it != optimized_order.end())
            {
                best_pos_it->first = best_dist_here_to_after;
            }
            optimized_order.insert(best_pos_it, std::make_pair(best_dist_before_to_here, to_be_inserted));

        }
        
        if (seam_config.type == EZSeamType::SHORTEST)
        { // only recompute the start when needed
            Point current_location = start_point;
            for (auto& [dist, path] : optimized_order)
            {
                path.start_vertex = findStartLocation(path, current_location);
                current_location = path.getStartLocation();
            }
        }

        //Apply the optimized order to the output field. Reverse if ordered to reverse.
        paths.clear();
        if (reverse_direction)
        {
            for (auto it = optimized_order.rbegin(); it != optimized_order.rend(); ++it)
            {
                paths.emplace_back(it->second);
            }
        }
        else
        {
            for (auto& [fist, path] : optimized_order)
            {
                paths.emplace_back(path);
            }
        }
    }

protected:
    /*!
     * If \ref detect_loops is enabled, endpoints of polylines that are closer
     * than this distance together will be considered to be coincident, closing
     * that polyline into a polygon.
     */
    constexpr static coord_t coincident_point_distance = 10;

    /*!
     * Some input data structures need to be converted to polygons before use.
     * For those, we need to store the vertex data somewhere during the lifetime
     * of the object. Store them here.
     *
     * For example, if the ``PathType`` is a list of ``ExtrusionJunction``s,
     * this will store the coordinates of those junctions.
     */
    std::vector<Polygon> cached_vertices;

    /*!
     * Bucket grid to store the locations of the combing boundary.
     *
     * This is cached in order to speed up the collision checking with the
     * combing boundary. We only need to generate this mapping once for the
     * combing boundary, since the combing boundary can't change.
     */
    std::unique_ptr<LocToLineGrid> combing_grid;

    /*!
     * Boundary to avoid when making travel moves.
     */
    const Polygons* combing_boundary;

    /*!
     * Whether to check polylines to see if they are closed, before optimizing.
     *
     * If this is enabled, the optimizer will first attempt to find endpoints of
     * polylines that are very close together. If they are closer than
     * \ref coincident_point_distance, the polylines will be closed and form
     * polygons. This then allows the optimizer to decide on a seam location
     * that is not one of the endpoints of the polyline.
     */
    bool detect_loops;

    /*!
     * Whether to reverse the ordering completely.
     *
     * This reverses the order in which parts are printed, and inverts the
     * direction of each path as well.
     */
    bool reverse_direction;

    /*!
     * The core algorithm to use:
     * - Selection sort based
     * - Insertion sort based
     * 
     * Selection does a greedy approach of finding the next best candidate to append to the result.
     * Insertion considers the best location within the result to insert each path.
     * 
     * Insertion sort can be wildly inefficient when polylines haven't been stitched.
     */
    bool selection_optimization;

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
    size_t findStartLocation(const Path& path, const Point& target_pos)
    {
        if(!path.is_closed)
        {
            //For polylines, the seam settings are not applicable. Simply choose the position closest to target_pos then.
            const coord_t back_distance = (combing_boundary == nullptr)
                ? getDirectDistance(path.converted->back(), target_pos)
                : getCombingDistance(path.converted->back(), target_pos);
            if(back_distance < getDirectDistance(path.converted->front(), target_pos) || (combing_boundary && back_distance < getCombingDistance(path.converted->front(), target_pos))) //Lazy or: Only compute combing distance if direct distance is closer.
            {
                return path.converted->size() - 1; //Back end is closer.
            }
            else
            {
                return 0; //Front end is closer.
            }
        }

        //Rest of the function only deals with (closed) polygons. We need to be able to find the seam location of those polygons.

        if(seam_config.type == EZSeamType::RANDOM)
        {
            size_t vert = getRandomPointInPolygon(*path.converted);
            return vert;
        }

        // Don't know the path-type here, or wether it has a simplify. Also, simplification occurs in-place, which is not wanted here: Copy the polygon.
        // A course simplification is needed, since Arachne has a tendency to 'smear' corners out over multiple line segments.
        // Which in itself is a good thing, but will mess up the detection of sharp corners and such.
        Polygon simple_poly(*path.converted);
        if (seam_config.simplify_curvature > 0 && simple_poly.size() > 2)
        {
            const coord_t max_simplify_dist2 = seam_config.simplify_curvature * seam_config.simplify_curvature;
            simple_poly.simplify(max_simplify_dist2, max_simplify_dist2 / 4);
        }
        if(simple_poly.empty()) //Simplify removed everything because it's all too small.
        {
            simple_poly = Polygon(*path.converted); //Restore the original. We have to output a vertex as the seam position, so there needs to be a vertex.
        }

        // Paths, other than polygons, can be either clockwise or counterclockwise. Make sure this is detected.
        const bool clockwise = simple_poly.orientation();

        //Find most extreme point in one direction. For the "actual loop" (see below), start from this point,
        //so it can act as a "tie breaker" if all differences in dist-score for a polygon fall within epsilon.
        //Direction/point should be the user-specified point if available, or an arbitrary point away from polygon otherwise.
        constexpr coord_t EPSILON = 25;
        const Point focus_fixed_point = (seam_config.type == EZSeamType::USER_SPECIFIED)
            ? seam_config.pos
            : Point(0, std::sqrt(std::numeric_limits<coord_t>::max())); //Use sqrt, so the squared size can be used when comparing distances.
        const size_t start_from_pos = std::min_element(simple_poly.begin(), simple_poly.end(), [focus_fixed_point](const Point& a, const Point& b) {
            return vSize2(a - focus_fixed_point) < vSize2(b - focus_fixed_point);
        }) - simple_poly.begin();
        const size_t end_before_pos = simple_poly.size() + start_from_pos;

        // Find a seam position in the simple polygon:
        Point best_point;
        float best_score = std::numeric_limits<float>::infinity();
        Point previous = simple_poly[(start_from_pos - 1 + simple_poly.size()) % simple_poly.size()];
        for(size_t i = start_from_pos; i < end_before_pos; ++i)
        {
            const Point& here = simple_poly[i % simple_poly.size()];
            const Point& next = simple_poly[(i + 1) % simple_poly.size()];

            //For most seam types, the shortest distance matters. Not for SHARPEST_CORNER though.
            //For SHARPEST_CORNER, use a fixed starting score of 0.
            const coord_t distance = (combing_boundary == nullptr)
                ? getDirectDistance(here, target_pos)
                : getCombingDistance(here, target_pos);
            const float score_distance = (seam_config.type == EZSeamType::SHARPEST_CORNER && seam_config.corner_pref != EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE) ? 0 : distance / 1000000;
            const float corner_angle = (clockwise ? LinearAlg2D::getAngleLeft(previous, here, next) : LinearAlg2D::getAngleLeft(next, here, previous)) / M_PI - 1; //Between -1 and 1.

            float score;
            const float corner_shift = seam_config.type != EZSeamType::USER_SPECIFIED ? 10000 : 0; //Allow up to 20mm shifting of the seam to find a good location. For SHARPEST_CORNER, this shift is the only factor. For USER_SPECIFIED, don't allow shifting.
            switch(seam_config.corner_pref)
            {
            default:
            case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_INNER:
                score = score_distance;
                if(corner_angle > 0) //Indeed a concave corner? Give it some advantage over other corners. More advantage for sharper corners.
                {
                    score -= (corner_angle + 1.0) * corner_shift;
                }
                break;
            case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_OUTER:
                score = score_distance;
                if(corner_angle < 0) //Indeed a convex corner?
                {
                    score -= (-corner_angle + 1.0) * corner_shift;
                }
                break;
            case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_ANY:
                score = score_distance - fabs(corner_angle) * corner_shift; //Still give sharper corners more advantage.
                break;
            case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE:
                score = score_distance; //No advantage for sharper corners.
                break;
            case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_WEIGHTED: //Give sharper corners some advantage, but sharper concave corners even more.
                {
                    float score_corner = fabs(corner_angle) * corner_shift;
                    if(corner_angle > 0) //Concave corner.
                    {
                        score_corner *= 2;
                    }
                    score = score_distance - score_corner;
                    break;
                }
            }

            if(seam_config.type == EZSeamType::USER_SPECIFIED) //If user-specified, the seam always needs to be the closest vertex that applies within the filter of the CornerPrefType. Give it a big penalty otherwise.
            {
                if((seam_config.corner_pref == EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_INNER && corner_angle <= 0)
                || (seam_config.corner_pref == EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_OUTER && corner_angle >= 0))
                {
                    score += 1000; //1 meter penalty.
                }
            }

            if(score - EPSILON < best_score)
            {
                best_point = here;
                best_score = score;
            }
            previous = here;
        }

        // Which point in the real deal is closest to the simple polygon version?
        size_t best_index = 0;
        coord_t closest_dist2 = std::numeric_limits<coord_t>::max();
        for (size_t i = 0; i < path.converted->size(); ++i)
        {
            const Point& here = (*path.converted)[i];
            const coord_t dist2 = vSize2(best_point - here);
            if (dist2 < closest_dist2)
            {
                closest_dist2 = dist2;
                best_index = i;
            }
        }

        return best_index;
    }

    /*!
     * Calculate the direct Euclidean distance to move from one point to
     * another.
     * \param a One point, to compute distance to \ref b.
     * \param b Another point, to compute distance to \ref a.
     * \return The distance between the two points.
     */
    coord_t getDirectDistance(const Point& a, const Point& b) const
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
    coord_t getCombingDistance(const Point& a, const Point& b)
    {
        if(!PolygonUtils::polygonCollidesWithLineSegment(*combing_boundary, a, b))
        {
            return getDirectDistance(a, b); //No collision with any line. Just compute the direct distance then.
        }
        if(paths.size() > 100)
        {
            /* If we have many paths to optimize the order for, this combing
            calculation can become very expensive. Instead, penalize travels
            that hit the combing boundary with a static factor.*/
            return getDirectDistance(a, b) * 5;
        }

        if(combing_grid == nullptr)
        {
            constexpr coord_t grid_size = 2000; //2mm grid cells. Smaller will use more memory, but reduce chance of unnecessary collision checks.
            combing_grid = PolygonUtils::createLocToLineGrid(*combing_boundary, grid_size);
        }

        CombPath comb_path; //Output variable.
        constexpr coord_t rounding_error = -25;
        constexpr coord_t tiny_travel_threshold = 0;
        constexpr bool fail_on_unavoidable_obstacles = false;
        LinePolygonsCrossings::comb(*combing_boundary, *combing_grid, a, b, comb_path, rounding_error, tiny_travel_threshold, fail_on_unavoidable_obstacles);

        coord_t sum = 0;
        Point last_point = a;
        for(const Point& point : comb_path)
        {
            sum += vSize(point - last_point);
            last_point = point;
        }
        return sum * sum; //Squared distance, for fair comparison with direct distance.
    }

    /*!
     * Get a random vertex of a polygon.
     * \param polygon A polygon to get a random vertex of.
     * \return A random index in that polygon.
     */
    size_t getRandomPointInPolygon(ConstPolygonRef const& polygon) const
    {
        return rand() % polygon.size();
    }

    bool isLoopingPolyline(const Path& path)
    {
        if(path.converted->empty())
        {
            return false;
        }
        return vSize2(path.converted->back() - path.converted->front()) < coincident_point_distance * coincident_point_distance;
    }

    /*!
     * Get vertex data from the custom path type.
     *
     * This is a function that allows the optimization algorithm to work with
     * any type of input data structure. It provides a translation from the
     * input data structure that the user would like to have ordered to a data
     * structure that the optimization algorithm can work with. It's unknown how
     * the ``PathType`` object is structured or how to get the vertex data from
     * it. This function tells the optimizer how, but it needs to be specialized
     * for each different type that this optimizer is used. See the .cpp file
     * for examples and where to add a new specialization.
     */
    ConstPolygonRef getVertexData(const PathType path);
};

} //namespace cura

#endif //PATHORDEROPTIMIZER_H
