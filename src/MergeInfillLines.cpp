//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/linearAlg2D.h" //Distance from point to line.
#include "MergeInfillLines.h"

namespace cura
{

    MergeInfillLines::MergeInfillLines(ExtruderPlan& plan) : extruder_plan(plan)
    {
        //Just copy the parameters to their fields.
    }

    /*
     * first_is_already_merged == false
     *
     *       o     o
     *      /     /
     *     /     /
     *    /  +  /      --->   -(t)-o-----o
     *   /     /
     *  /     /
     * o     o
     *
     * travel (t) to first location is done through first_path_start_changed and new_first_path_start.
     * this gets rid of the tiny "blips". Depending on the merged line distance a small gap may appear, but this is
     * accounted for in the volume.
     *
     * first_is_already_merged == true
     *
     *                   o
     *                  /
     *                 /
     *    o-----o  +  /     --->    o-----------o   or with slight   o-----o-----o
     *   /           /             /                   bend         /
     *  /           /             /                                /
     * o           o             o                                o
     *
     */
    void MergeInfillLines::mergeLinesSideBySide(const bool first_is_already_merged, GCodePath& first_path, const Point first_path_start, GCodePath& second_path, const Point second_path_start, Point& new_first_path_start) const
    {
        coord_t first_path_length_flow = 0;
        Point average_first_path;

        Point previous_point = first_path_start;
        for (const Point point : first_path.points)
        {
            first_path_length_flow += vSize(point - previous_point);
            previous_point = point;
        }
        first_path_length_flow *= first_path.flow; //To get the volume we don't need to include the line width since it's the same for both lines.

        if (first_is_already_merged)
        {
            // take second point of path, first_path.points[0]
            average_first_path = first_path.points[0];
        }
        else
        {
            average_first_path += first_path_start;
            for (const Point point : first_path.points)
            {
                average_first_path += point;
            }
            average_first_path /= (first_path.points.size() + 1);
        }

        coord_t second_path_length_flow = 0;
        Point previous_point_second = second_path_start;
        Point average_second_path = second_path_start;
        for (const Point point : second_path.points)
        {
            second_path_length_flow += vSize(point - previous_point_second);
            average_second_path += point;
            previous_point_second = point;
        }
        second_path_length_flow *= second_path.flow;
        average_second_path /= (second_path.points.size() + 1);

        coord_t new_path_length = 0;

        if (first_is_already_merged)
        {
            // check if the new point is a good extension of last part of existing polyline
            // because of potential accumulation of errors introduced each time a line is merged, we do not allow any error.
            if (first_path.points.size() > 1 && LinearAlg2D::getDist2FromLine(average_second_path, first_path.points[first_path.points.size()-2], first_path.points[first_path.points.size()-1]) == 0) {
                first_path.points[first_path.points.size()-1] = average_second_path;
            } else {
                first_path.points.push_back(average_second_path);
            }
        }
        else
        {
            first_path.points.clear();
            new_first_path_start = average_first_path;
            //first_path.points.push_back(average_first_path);
            first_path.points.push_back(average_second_path);
        }
        previous_point = first_path_start;
        for (const Point point : first_path.points)
        {
            new_path_length += vSize(point - previous_point);
            previous_point = point;
        }

        first_path.flow = static_cast<double>(first_path_length_flow + second_path_length_flow) / new_path_length;
    }


    bool MergeInfillLines::tryMerge(const bool first_is_already_merged, GCodePath& first_path, const Point first_path_start, GCodePath& second_path, const Point second_path_start, Point& new_first_path_start) const
    {
        const Point first_path_end = first_path.points.back();
        const Point second_path_end = second_path.points.back();
        const coord_t line_width = first_path.config->getLineWidth();

        //Lines may be adjacent side-by-side then.
        Point first_path_leave_point;
        coord_t merged_size2;
        if (first_is_already_merged)
        {
            first_path_leave_point = first_path.points.back();  // this is the point that's going to merge
        } else {
            first_path_leave_point = (first_path_start + first_path_end) / 2;
        }
        const Point second_path_destination_point = (second_path_start + second_path_end) / 2;
        const Point merged_direction = second_path_destination_point - first_path_leave_point;
        if (first_is_already_merged)
        {
            merged_size2 = vSize2(second_path_destination_point - first_path.points.back());  // check distance with last point in merged line that is to be replaced
        }
        else
        {
            merged_size2 = vSize2(merged_direction);
        }
        if (merged_size2 > 25 * line_width * line_width)
        {
            return false; //Lines are too far away from each other.
        }
        if (merged_direction.X == 0 && merged_direction.Y == 0)
        {
            return true;  // we can just disregard the second point as it's exactly at the leave point of the first path.
        }

        // Max 1 line width to the side of the merged_direction
        if (LinearAlg2D::getDist2FromLine(first_path_end, second_path_destination_point, second_path_destination_point + merged_direction) > line_width * line_width
            || LinearAlg2D::getDist2FromLine(second_path_start, first_path_leave_point, first_path_leave_point + merged_direction) > line_width * line_width
            || LinearAlg2D::getDist2FromLine(second_path_end,   first_path_leave_point, first_path_leave_point + merged_direction) > line_width * line_width
            || abs(dot(normal(merged_direction, 1000), normal(second_path_end - second_path_start, 1000))) > 866000)  // angle of old second_path with new merged direction should not be too small (30 degrees), as it will introduce holes
        {
            return false; //One of the lines is too far from the merged line. Lines would be too wide or too far off.
        }
        if (first_is_already_merged && first_path.points.size() > 1 && first_path.points[first_path.points.size() - 2] == second_path_destination_point)  // yes this can actually happen
        {
            return false;
        }
        mergeLinesSideBySide(first_is_already_merged, first_path, first_path_start, second_path, second_path_start, new_first_path_start);
        return true;
    }


    bool MergeInfillLines::mergeInfillLines(std::vector<GCodePath>& paths, const Point& starting_position) const
    {
        /* Algorithm overview:
            1. Loop over all lines to see if they can be merged.
            1a. Check if two adjacent lines can be merged (skipping travel moves in
                between).
            1b. If they can, merge both lines into the first line.
            1c. If they are merged, check next that the first line can be merged
                with the line after the second line.
            2. Do a second iteration over all paths to remove the tombstones. */
        std::vector<size_t> remove_path_indices;
        std::set<size_t> is_merged;
        std::set<size_t> removed;  // keep track of what we already removed, so don't remove it again

        //For each two adjacent lines, see if they can be merged.
        size_t first_path_index = 0;
        Point first_path_start = Point(starting_position.X, starting_position.Y);  // this one is not going to be overwritten
        size_t second_path_index = 1;

        for (; second_path_index < paths.size(); second_path_index++)
        {
            GCodePath& first_path = paths[first_path_index];
            GCodePath& second_path = paths[second_path_index];
            Point second_path_start = paths[second_path_index - 1].points.back();
            const coord_t line_width = first_path.config->getLineWidth();

            if (second_path.config->isTravelPath())
            {
                continue; //Skip travel paths, we're looking for the first non-travel path.
            }

            bool allow_try_merge = true;
            // see if we meet criteria to merge. should be: travel - path1 not travel - (...) - travel - path2 not travel - travel
            // we're checking the travels here
            if (first_path_index <= 1 || !paths[first_path_index - 1].isTravelPath())  // "<= 1" because we don't want the first travel being changed. That may introduce a hole somewhere
            {
                allow_try_merge = false;
            }
            if (second_path_index + 1 >= paths.size() || !paths[second_path_index + 1].isTravelPath())
            {
                allow_try_merge = false;
            }
            if (first_path.config->isTravelPath()) //Don't merge travel moves.
            {
                allow_try_merge = false;
            }
            if (first_path.config != second_path.config) //Only merge lines that have the same type.
            {
                allow_try_merge = false;
            }
            if (first_path.config->type != PrintFeatureType::Infill && first_path.config->type != PrintFeatureType::Skin) //Only merge skin and infill lines.
            {
                allow_try_merge = false;
            }
            const bool first_is_already_merged = is_merged.find(first_path_index) != is_merged.end();
            if ((!first_is_already_merged && first_path.points.size() > 1) || second_path.points.size() > 1)
            {
                // For now we only merge simple lines, not polylines, to keep it simple.
                // If the first line is already a merged line, then allow it.
                allow_try_merge = false;
            }

            Point new_first_path_start;
            if (allow_try_merge && tryMerge(first_is_already_merged, first_path, first_path_start, second_path, second_path_start, new_first_path_start))
            {
                if (!first_is_already_merged)
                {
                    paths[first_path_index - 1].points.back().X = new_first_path_start.X;
                    paths[first_path_index - 1].points.back().Y = new_first_path_start.Y;
                }
                /* If we combine two lines, the next path may also be merged into the fist line, so we do NOT update
                first_path_index. */
                for (size_t to_delete_index = first_path_index + 1; to_delete_index <= second_path_index; to_delete_index++)
                {
                    if (removed.find(to_delete_index) == removed.end())  // if there are line(s) between first and second, then those lines are already marked as to be deleted, only add the new line(s)
                    {
                        remove_path_indices.push_back(to_delete_index);
                        removed.insert(to_delete_index);
                    }
                }
                is_merged.insert(first_path_index);
            }
            else
            {
                /* If we do not combine, the next iteration we must simply merge the
                second path with the line after it. */
                first_path_index = second_path_index;
                first_path_start = second_path_start;
            }
        }

        //Delete all removed lines in one pass so that we need to move lines less often.
        if (!remove_path_indices.empty())
        {
            size_t path_index = remove_path_indices[0];
            for (size_t removed_position = 1; removed_position < remove_path_indices.size(); removed_position++)
            {
                for (; path_index < remove_path_indices[removed_position] - removed_position; path_index++)
                {
                    paths[path_index] = paths[path_index + removed_position]; //Shift all paths.
                }
            }
            for (; path_index < paths.size() - remove_path_indices.size(); path_index++) //Remaining shifts at the end.
            {
                paths[path_index] = paths[path_index + remove_path_indices.size()];
            }
            paths.erase(paths.begin() + path_index, paths.end());
            return true;
        }
        else
        {
            return false;
        }
    }
}//namespace cura
