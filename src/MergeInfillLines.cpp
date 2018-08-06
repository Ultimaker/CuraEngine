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

bool MergeInfillLines::mergeInfillLines(std::vector<GCodePath>& paths, const Point starting_position) const
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
    std::set<size_t> removed;  // keep track of what we already removed, so don't remove it again

    //For each two adjacent lines, see if they can be merged.
    size_t first_path_index = 0;
    Point first_path_start = starting_position;
    size_t second_path_index = 1;

    for (; second_path_index < paths.size(); second_path_index++)
    {
        GCodePath& first_path = paths[first_path_index];
        GCodePath& second_path = paths[second_path_index];
        Point second_path_start = paths[second_path_index - 1].points.back();
        if (second_path.config->isTravelPath())
        {
            continue; //Skip travel paths.
        }

        if (isConvertible(first_path, first_path_start, second_path, second_path_start))
        {
            /* If we combine two lines, the second path is inside the first
            line, so the iteration after that we need to merge the first line
            with the line after the second line, so we do NOT update
            first_path_index. */
            mergeLines(first_path, first_path_start, second_path, second_path_start);
            for (size_t to_delete_index = first_path_index + 1; to_delete_index <= second_path_index; to_delete_index++)
            {
                if (removed.find(to_delete_index) == removed.end())  // if there are line(s) between first and second, then those lines are already marked as to be deleted, only add the new line(s)
                {
                    remove_path_indices.push_back(to_delete_index);
                    removed.insert(to_delete_index);
                }
            }
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

bool MergeInfillLines::isConvertible(const GCodePath& first_path, Point first_path_start, const GCodePath& second_path, const Point second_path_start) const
{
    if (first_path.config->isTravelPath()) //Don't merge travel moves.
    {
        return false;
    }
    if (first_path.config != second_path.config) //Only merge lines that have the same type.
    {
        return false;
    }
    if (first_path.config->type != PrintFeatureType::Infill && first_path.config->type != PrintFeatureType::Skin) //Only merge skin and infill lines.
    {
        return false;
    }
    if (first_path.points.size() > 1 || second_path.points.size() > 1)
    {
        //TODO: For now we only merge simple lines, not polylines, to keep it simple.
        return false;
    }

    Point first_path_end = first_path.points.back();
    const Point second_path_end = second_path.points.back();
    Point first_direction = first_path_end - first_path_start;
    const Point second_direction = second_path_end - second_path_start;
    coord_t dot_product = dot(first_direction, second_direction);
    const coord_t first_size = vSize(first_direction);
    const coord_t second_size = vSize(second_direction);
    if (dot_product < 0) //Make lines in the same direction by flipping one.
    {
        first_direction *= -1;
        dot_product *= -1;
        Point swap = first_path_end;
        first_path_end = first_path_start;
        first_path_start = swap;
    }

    //Check if lines are connected end-to-end and can be merged that way.
    const coord_t line_width = first_path.config->getLineWidth();
    if (vSize2(first_path_end - second_path_start) < line_width * line_width || vSize2(first_path_start - second_path_end) < line_width * line_width) //Paths are already (practically) connected, end-to-end.
    {
        //Only merge if lines are more or less in the same direction.
        return dot_product + 400 > first_size * second_size; //400 = 20*20, where 20 micron is the allowed inaccuracy in the dot product, allowing a slight curve.
    }

    //Lines may be adjacent side-by-side then.
    const Point first_path_middle = (first_path_start + first_path_end) / 2;
    const Point second_path_middle = (second_path_start + second_path_end) / 2;
    const Point merged_direction = second_path_middle - first_path_middle;
    const coord_t merged_size2 = vSize2(merged_direction);
    if (merged_size2 > 25 * line_width * line_width)
    {
        return false; //Lines are too far away from each other.
    }
    if (LinearAlg2D::getDist2FromLine(first_path_start,  second_path_middle, second_path_middle + merged_direction) > 4 * line_width * line_width
     || LinearAlg2D::getDist2FromLine(first_path_end,    second_path_middle, second_path_middle + merged_direction) > 4 * line_width * line_width
     || LinearAlg2D::getDist2FromLine(second_path_start, first_path_middle,  first_path_middle  + merged_direction) > 4 * line_width * line_width
     || LinearAlg2D::getDist2FromLine(second_path_end,   first_path_middle,  first_path_middle  + merged_direction) > 4 * line_width * line_width)
    {
        return false; //One of the lines is too far from the merged line. Lines would be too wide or too far off.
    }

    return true;
}

void MergeInfillLines::mergeLines(GCodePath& first_path, const Point first_path_start, const GCodePath& second_path, const Point second_path_start) const
{
    //We may apply one of two merging techniques: Append the second path to the first, or draw a line through the middle of both of them.
    const coord_t line_width = first_path.config->getLineWidth();
    if (vSize2(first_path.points.back() - second_path_start) < line_width * line_width || vSize2(first_path_start - second_path.points.back()) < line_width * line_width)
    {
        for (const Point second_path_point : second_path.points)
        {
            first_path.points.push_back(second_path_point); //Move the coordinates over to the first path.
        }
        return;
    }

    //Alternative: Merge adjacent lines by drawing a line through them.
    coord_t first_path_length = 0;
    Point previous_point = first_path_start;
    Point average_first_path;
    for (const Point point : first_path.points)
    {
        first_path_length += vSize(point - previous_point);
        average_first_path += point;
    }
    first_path_length *= first_path.flow; //To get the volume we don't need to include the line width since it's the same for both lines.
    average_first_path /= first_path.points.size();

    coord_t second_path_length = 0;
    previous_point = second_path_start;
    Point average_second_path;
    for (const Point point : second_path.points)
    {
        second_path_length += vSize(point - previous_point);
        average_second_path += point;
    }
    second_path_length *= second_path.flow;
    average_second_path /= second_path.points.size();

    first_path.points.clear();
    first_path.points.push_back(average_second_path);
    const coord_t new_path_length = vSize(average_second_path - first_path_start);
    first_path.flow = static_cast<double>(first_path_length + second_path_length) / new_path_length;
}

}//namespace cura
