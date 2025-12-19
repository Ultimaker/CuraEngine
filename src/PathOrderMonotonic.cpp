// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PathOrderMonotonic.h"

#include <unordered_map> //To track monotonic sequences.
#include <unordered_set> //To track starting points of monotonic sequences.

#include "geometry/OpenPolyline.h"

namespace cura
{

template<typename PathType>
void PathOrderMonotonic<PathType>::optimize()
{
    if (this->paths_.empty())
    {
        return;
    }

    // Get the vertex data and store it in the paths.
    for (Path& path : this->paths_)
    {
        path.converted_ = &path.getVertexData();
    }

    std::vector<Path> reordered; // To store the result in. At the end, we'll std::swap with the real paths.
    reordered.reserve(this->paths_.size());

    // First print all the looping polygons, if there are any.
    std::vector<Path*> polylines; // Also find all polylines and store them in a vector that we can sort in-place without making copies all the time.
    this->detectLoops(); // Always filter out loops. We don't specifically want to print those in monotonic order.
    for (Path& path : this->paths_)
    {
        if (path.is_closed_ || path.vertices_->size() <= 1)
        {
            reordered.push_back(path);
        }
        else
        {
            polylines.push_back(&path);
            // Assign an invalid starting vertex to indicate we don't know the starting point yet.
            polylines.back()->start_vertex_ = polylines.back()->converted_->size();
        }
    }

    // Sort the polylines by their projection on the monotonic vector. This helps find adjacent lines quickly.
    std::stable_sort(
        polylines.begin(),
        polylines.end(),
        [this](Path* a, Path* b)
        {
            const double a_start_projection = projectToMonotonicVector(a->converted_->front());
            const double a_end_projection = projectToMonotonicVector(a->converted_->back());
            const double a_projection = std::min(a_start_projection, a_end_projection); // The projection of a path is the endpoint furthest back of the two endpoints.

            const double b_start_projection = projectToMonotonicVector(b->converted_->front());
            const double b_end_projection = projectToMonotonicVector(b->converted_->back());
            const double b_projection = std::min(b_start_projection, b_end_projection);

            return a_projection < b_projection;
        });

    if (interlaced_)
    {
        // Separate the lines in two sets by adjacency
        std::vector<Path*> lines_pass1;
        std::vector<Path*> lines_pass2;
        lines_pass1.reserve(polylines.size() / 2); // Rough estimation
        lines_pass2.reserve(polylines.size() / 2);

        std::vector<Path*>* current_pass = nullptr;
        double current_projection = 0;

        for (Path* path : polylines)
        {
            const double path_projection = projectToMonotonicVector(path->converted_->front());
            if (current_pass == nullptr)
            {
                current_pass = &lines_pass1;
                current_projection = path_projection;
            }
            else if ((path_projection - current_projection) > (max_adjacent_distance_ / 2))
            {
                current_pass = current_pass == &lines_pass1 ? &lines_pass2 : &lines_pass1;
                current_projection = path_projection;
            }

            current_pass->push_back(path);
        }

        // Process the 2 sets independently and concatenate them to get the final 2-pass monotonic order
        std::vector<Path> ordered_lines_pass1 = makeOrderedPath(lines_pass1, max_adjacent_distance_ * 2);
        std::vector<Path> ordered_lines_pass2 = makeOrderedPath(lines_pass2, max_adjacent_distance_ * 2);

        reordered.insert(reordered.begin(), std::make_move_iterator(ordered_lines_pass1.begin()), std::make_move_iterator(ordered_lines_pass1.end()));
        reordered.insert(reordered.begin(), std::make_move_iterator(ordered_lines_pass2.begin()), std::make_move_iterator(ordered_lines_pass2.end()));
    }
    else
    {
        std::vector<Path> lines_ordered = makeOrderedPath(polylines, max_adjacent_distance_);
        reordered.insert(reordered.begin(), std::make_move_iterator(lines_ordered.begin()), std::make_move_iterator(lines_ordered.end()));
    }

    std::swap(reordered, this->paths_); // Store the resulting list in the main paths.
}

template<typename PathType>
std::deque<typename PathOrderMonotonic<PathType>::Path*> PathOrderMonotonic<PathType>::findPolylineString(Path* polyline, const SparsePointGridInclusive<Path*>& line_bucket_grid)
{
    std::deque<Path*> result;
    if (polyline->converted_->empty())
    {
        return result;
    }

    // Find the two endpoints of the polyline string, on either side.
    result.push_back(polyline);
    polyline->start_vertex_ = 0;
    Point2LL first_endpoint = polyline->converted_->front();
    Point2LL last_endpoint = polyline->converted_->back();
    std::vector<SparsePointGridInclusiveImpl::SparsePointGridInclusiveElem<Path*>> lines_before = line_bucket_grid.getNearby(first_endpoint, coincident_point_distance_);
    auto close_line_before = std::find_if(
        lines_before.begin(),
        lines_before.end(),
        [first_endpoint](SparsePointGridInclusiveImpl::SparsePointGridInclusiveElem<Path*> found_path)
        {
            return canConnectToPolyline(first_endpoint, found_path);
        });
    std::vector<SparsePointGridInclusiveImpl::SparsePointGridInclusiveElem<Path*>> lines_after = line_bucket_grid.getNearby(last_endpoint, coincident_point_distance_);
    auto close_line_after = std::find_if(
        lines_after.begin(),
        lines_after.end(),
        [last_endpoint](SparsePointGridInclusiveImpl::SparsePointGridInclusiveElem<Path*> found_path)
        {
            return canConnectToPolyline(last_endpoint, found_path);
        });

    while (close_line_before != lines_before.end())
    {
        Path* first = close_line_before->val;
        result.push_front(first); // Store this one in the sequence. It's a good one.
        size_t farthest_vertex = getFarthestEndpoint(first, close_line_before->point); // Get to the opposite side.
        first->start_vertex_ = farthest_vertex;
        first->backwards_ = farthest_vertex != 0;
        first_endpoint = (*first->converted_)[farthest_vertex];
        lines_before = line_bucket_grid.getNearby(first_endpoint, coincident_point_distance_);
        close_line_before = std::find_if(
            lines_before.begin(),
            lines_before.end(),
            [first_endpoint](SparsePointGridInclusiveImpl::SparsePointGridInclusiveElem<Path*> found_path)
            {
                return canConnectToPolyline(first_endpoint, found_path);
            });
    }
    while (close_line_after != lines_after.end())
    {
        Path* last = close_line_after->val;
        result.push_back(last);
        size_t farthest_vertex = getFarthestEndpoint(last, close_line_after->point); // Get to the opposite side.
        last->start_vertex_ = (farthest_vertex == 0) ? last->converted_->size() - 1 : 0;
        last->backwards_ = farthest_vertex != 0;
        last_endpoint = (*last->converted_)[farthest_vertex];
        lines_after = line_bucket_grid.getNearby(last_endpoint, coincident_point_distance_);
        close_line_after = std::find_if(
            lines_after.begin(),
            lines_after.end(),
            [last_endpoint](SparsePointGridInclusiveImpl::SparsePointGridInclusiveElem<Path*> found_path)
            {
                return canConnectToPolyline(last_endpoint, found_path);
            });
    }

    // Figure out which of the two endpoints to start with: The one monotonically earliest.
    const double first_projection = projectToMonotonicVector(first_endpoint);
    const double last_projection = projectToMonotonicVector(last_endpoint);
    // If the last endpoint should be printed first (unlikely due to monotonic start, but possible), flip the whole polyline!
    if (last_projection < first_projection)
    {
        std::reverse(result.begin(), result.end());
        for (Path* path : result) // Also reverse their start_vertex.
        {
            path->start_vertex_ = (path->start_vertex_ == 0) ? path->converted_->size() - 1 : 0;
            path->backwards_ = ! path->backwards_;
        }
    }

    if (result.size() == 1)
    {
        result[0]->start_vertex_ = result[0]->converted_->size(); // Reset start vertex as "unknown" again if it's not a string of polylines.
    }
    return result;
}

template<typename PathType>
std::vector<typename PathOrderMonotonic<PathType>::Path*> PathOrderMonotonic<PathType>::getOverlappingLines(
    const typename std::vector<Path*>::const_iterator& polyline_it,
    const Point2D& perpendicular,
    const std::vector<Path*>& polylines,
    const coord_t max_adjacent_distance)
{
    // How far this extends in the monotonic direction, to make sure we only go up to max_adjacent_distance in that direction.
    const double start_monotonic = projectToMonotonicVector((*polyline_it)->converted_->front());
    const double end_monotonic = projectToMonotonicVector((*polyline_it)->converted_->back());
    const double my_farthest_monotonic = std::max(start_monotonic, end_monotonic);
    const double my_closest_monotonic = std::min(start_monotonic, end_monotonic);
    const double my_farthest_monotonic_padded = my_farthest_monotonic + max_adjacent_distance;
    const double my_closest_monotonic_padded = my_closest_monotonic - max_adjacent_distance;
    // How far this line reaches in the perpendicular direction -- the range at which the line overlaps other lines.
    const double my_start = projectToVector((*polyline_it)->converted_->front(), perpendicular);
    const double my_end = projectToVector((*polyline_it)->converted_->back(), perpendicular);
    const double my_farthest = std::max(my_start, my_end);
    const double my_closest = std::min(my_start, my_end);
    const double my_farthest_padded = my_farthest + max_adjacent_distance;
    const double my_closest_padded = my_closest - max_adjacent_distance;

    std::vector<Path*> overlapping_lines;
    for (auto overlapping_line = polyline_it + 1; overlapping_line != polylines.end(); ++overlapping_line)
    {
        // Don't go beyond the maximum adjacent distance.
        const double start_their_projection = projectToMonotonicVector((*overlapping_line)->converted_->front());
        const double end_their_projection = projectToMonotonicVector((*overlapping_line)->converted_->back());
        const double their_farthest_projection = std::max(start_their_projection, end_their_projection);
        const double their_closest_projection = std::min(start_their_projection, end_their_projection);
        // Multiply by the length of the vector since we need to compare actual distances here.
        if (their_closest_projection > my_farthest_monotonic_padded || my_closest_monotonic_padded > their_farthest_projection)
        {
            break; // Too far. This line and all subsequent lines are not adjacent anymore, even though they might be side-by-side.
        }

        // Does this one overlap?
        const double their_start = projectToVector((*overlapping_line)->converted_->front(), perpendicular);
        const double their_end = projectToVector((*overlapping_line)->converted_->back(), perpendicular);
        const double their_farthest = std::max(their_start, their_end);
        const double their_closest = std::min(their_start, their_end);
        /*There are 5 possible cases of overlapping:
        - We are behind them, partially overlapping. my_start is between their_start and their_end.
        - We are in front of them, partially overlapping. my_end is between their_start and their_end.
        - We are a smaller line, they completely overlap us. Both my_start and my_end are between their_start and their_end. (Caught with the first 2 conditions already.)
        - We are a bigger line, and completely overlap them. Both their_start and their_end are between my_start and my_end.
        - Lines are exactly equal. Start and end are the same. (Caught with the previous condition too.)*/
        if ((my_closest_padded >= their_closest && my_closest_padded <= their_farthest) || (my_farthest_padded >= their_closest && my_farthest_padded <= their_farthest)
            || (their_closest >= my_closest_padded && their_farthest <= my_farthest_padded))
        {
            overlapping_lines.push_back(*overlapping_line);
        }
    }

    return overlapping_lines;
}

template<typename PathType>
std::vector<typename PathOrderMonotonic<PathType>::Path> PathOrderMonotonic<PathType>::makeOrderedPath(const std::vector<Path*>& polylines, const coord_t max_adjacent_distance)
{
    std::vector<Path> reordered; // To store the result in. At the end, we'll std::swap with the real paths.
    reordered.reserve(polylines.size());

    // Create a bucket grid to be able to find adjacent lines quickly.
    SparsePointGridInclusive<Path*> line_bucket_grid(MM2INT(2)); // Grid size of 2mm.
    for (Path* polyline : polylines)
    {
        if (! polyline->converted_->empty())
        {
            line_bucket_grid.insert(polyline->converted_->front(), polyline);
            line_bucket_grid.insert(polyline->converted_->back(), polyline);
        }
    }

    // Create sequences of line segments that get printed together in a monotonic direction.
    // There are several constraints we impose here:
    //  - Strings of incident polylines are printed in sequence. That is, if their endpoints are incident.
    //    - The endpoint of the string that is earlier in the monotonic direction will get printed first.
    //    - The start_vertex of this line will already be set to indicate where to start from.
    //  - If a line overlaps with another line in the perpendicular direction, and is within max_adjacent_distance (~1 line width) in the monotonic direction, it must be
    //  printed in monotonic order.
    //    - The earlier line is marked as being in sequence with the later line.
    //    - The later line is no longer a starting point, unless there are multiple adjacent lines before it.
    // The ``starting_lines`` set indicates possible locations to start from. Each starting line represents one "sequence", which is either a set of adjacent line segments or a
    // string of polylines. The ``connections`` map indicates, starting from each starting segment, the sequence of line segments to print in order. Note that for performance
    // reasons, the ``connections`` map will sometimes link the end of one segment to the start of the next segment. This link should be ignored.
    const Point2D perpendicular = monotonic_vector_.rotated90CCW(); // To project on to detect adjacent lines.

    std::unordered_set<Path*> connected_lines; // Lines that are reachable from one of the starting lines through its connections.
    std::unordered_set<Path*> starting_lines; // Starting points of a linearly connected segment.
    std::unordered_map<Path*, Path*> connections; // For each polyline, which polyline it overlaps with, closest in the projected order.

    for (auto polyline_it = polylines.begin(); polyline_it != polylines.end(); polyline_it++)
    {
        if (connections.contains(*polyline_it)) // Already connected this one through a polyline.
        {
            continue;
        }
        // First find out if this polyline is part of a string of polylines.
        std::deque<Path*> polystring = findPolylineString(*polyline_it, line_bucket_grid);

        // If we're part of a string of polylines, connect up the whole string and mark all of them as being connected.
        if (polystring.size() > 1)
        {
            starting_lines.insert(polystring[0]);
            for (size_t i = 0; i < polystring.size() - 1; ++i) // Iterate over every pair of adjacent polylines in the string (so skip the last one)!
            {
                connections[polystring[i]] = polystring[i + 1];
                connected_lines.insert(polystring[i + 1]);

                // Even though we chain polylines, we still want to find lines that they overlap with.
                // The strings of polylines may still have weird shapes which interweave with other strings of polylines or loose lines.
                // So when a polyline string comes into contact with other lines, we still want to guarantee their order.
                // So here we will look for which lines they come into contact with, and thus mark those as possible starting points, so that they function as a new junction.
                const std::vector<Path*> overlapping_lines
                    = getOverlappingLines(std::find(polylines.begin(), polylines.end(), polystring[i]), perpendicular, polylines, max_adjacent_distance);
                for (Path* overlapping_line : overlapping_lines)
                {
                    if (std::find(polystring.begin(), polystring.end(), overlapping_line)
                        == polystring.end()) // Mark all overlapping lines not part of the string as possible starting points.
                    {
                        starting_lines.insert(overlapping_line);
                        starting_lines.insert(polystring[i + 1]); // Also be able to re-start from this point in the string.
                    }
                }
            }
        }
        else // Not a string of polylines, but simply adjacent line segments.
        {
            if (! connected_lines.contains(*polyline_it)) // Nothing connects to this line yet.
            {
                starting_lines.insert(*polyline_it); // This is a starting point then.
            }
            const std::vector<Path*> overlapping_lines = getOverlappingLines(polyline_it, perpendicular, polylines, max_adjacent_distance);
            if (overlapping_lines.size() == 1) // If we're not a string of polylines, but adjacent to only one other polyline, create a sequence of polylines.
            {
                connections[*polyline_it] = overlapping_lines[0];
                if (connected_lines.contains(overlapping_lines[0])) // This line was already connected to.
                {
                    starting_lines.insert(overlapping_lines[0]); // Multiple lines connect to it, so we must be able to start there.
                }
                else
                {
                    connected_lines.insert(overlapping_lines[0]);
                }
            }
            else // Either 0 (the for loop terminates immediately) or multiple overlapping lines. For multiple lines we need to mark all of them a starting position.
            {
                for (Path* overlapping_line : overlapping_lines)
                {
                    starting_lines.insert(overlapping_line);
                }
            }
        }
    }

    struct LineProjections
    {
        coord_t min;
        coord_t max;

        explicit LineProjections(Path* path, const Point2D& monotonic_vector)
        {
            const coord_t start_projection = std::llround(projectToVector(path->converted_->front(), monotonic_vector) / precision_factor);
            const coord_t end_projection = std::llround(projectToVector(path->converted_->back(), monotonic_vector) / precision_factor);
            std::tie(min, max) = std::minmax(start_projection, end_projection);
        }

        bool operator<(const LineProjections& other) const
        {
            return min < other.min || (min == other.min && max < other.max);
        }
    };

    // Pre-order lines in a multi-map, so that aligned lines will end up in the same bucket and we can process them in a row
    std::multimap<LineProjections, Path*> pre_ordered_lines;
    for (Path* starting_line : starting_lines)
    {
        pre_ordered_lines.insert(std::make_pair(LineProjections(starting_line, monotonic_vector_), starting_line));
    }

    // Now order the lines, by finding the closest line from the current position in the current bucket (row)
    Point2LL current_pos = this->start_point_;
    while (! pre_ordered_lines.empty())
    {
        const LineProjections first_projection_key = pre_ordered_lines.begin()->first;
        auto lines_on_row_range = pre_ordered_lines.equal_range(first_projection_key);

        coord_t closest_distance = std::numeric_limits<coord_t>::max();
        auto closest_next_line_iterator = lines_on_row_range.second;
        bool closest_backwards = false;
        for (auto iterator = lines_on_row_range.first; iterator != lines_on_row_range.second; ++iterator)
        {
            const Path* path = iterator->second;
            const coord_t dist_start = vSize2(current_pos - path->converted_->front());
            const coord_t dist_end = vSize2(current_pos - path->converted_->back());
            if (dist_start < closest_distance)
            {
                closest_next_line_iterator = iterator;
                closest_distance = dist_start;
                closest_backwards = false;
            }
            if (dist_end < closest_distance)
            {
                closest_next_line_iterator = iterator;
                closest_distance = dist_end;
                closest_backwards = true;
            }
        }

        Path* closest_path = closest_next_line_iterator->second;
        setStartVertex(closest_path, closest_backwards);
        current_pos = (*closest_path->converted_)[closest_path->converted_->size() - 1 - closest_path->start_vertex_]; // Opposite of the start vertex.
        reordered.push_back(*closest_path);
        pre_ordered_lines.erase(closest_next_line_iterator);

        // Now add the (adjacent) lines to be processed together with this one
        auto connection = connections.find(closest_path);
        std::unordered_map<Path*, Path*> checked_connections; // Which connections have already been iterated over
        auto checked_connection = checked_connections.find(closest_path);

        while (connection != connections.end() // Stop if the sequence ends
               && starting_lines.find(connection->second) == starting_lines.end() // or if we hit another starting point.
               && (checked_connection == checked_connections.end()
                   || checked_connection->second != connection->second)) // or if we have already checked the connection (to avoid falling into a cyclical connection)
        {
            checked_connections.insert({ connection->first, connection->second });
            Path* line = connection->second;
            optimizeClosestStartPoint(line, current_pos);
            reordered.push_back(*line); // Plan this line in, to be printed next!
            connection = connections.find(line);
            checked_connection = checked_connections.find(line);
        }
    }

    return reordered;
}

// Template functions instantiations
template void PathOrderMonotonic<const Polyline*>::optimize();
template void PathOrderMonotonic<const OpenPolyline*>::optimize();

} // namespace cura
