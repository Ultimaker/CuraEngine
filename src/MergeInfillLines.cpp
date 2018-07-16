//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "MergeInfillLines.h"

namespace cura
{

MergeInfillLines::MergeInfillLines(ExtruderPlan& plan) : extruder_plan(plan)
{
    //Just copy the parameters to their fields.
}

bool MergeInfillLines::mergeInfillLines(std::vector<GCodePath>& paths) const
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

    //For each two adjacent lines, see if they can be merged.
    size_t first_path_index = 0;
    size_t second_path_index = 1;
    for (; second_path_index < paths.size(); second_path_index++)
    {
        GCodePath& first_path = paths[first_path_index];
        GCodePath& second_path = paths[second_path_index];
        if (second_path.config->isTravelPath())
        {
            continue; //Skip travel paths.
        }

        if (isConvertible(first_path, second_path))
        {
            /* If we combine two lines, the second path is inside the first
            line, so the iteration after that we need to merge the first line
            with the line after the second line, so we do NOT update
            first_path_index. */
            mergeLines(first_path, second_path);
            for (size_t to_delete_index = first_path_index + 1; to_delete_index <= second_path_index; to_delete_index++)
            {
                remove_path_indices.push_back(to_delete_index);
            }
        }
        else
        {
            /* If we do not combine, the next iteration we must simply merge the
            second path with the line after it. */
            first_path_index = second_path_index;
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

bool MergeInfillLines::isConvertible(GCodePath& first_path, GCodePath& second_path) const
{
    return false; //TODO.
}

void MergeInfillLines::mergeLines(GCodePath& first_path, const GCodePath& second_path) const
{
    //TODO.
}

}//namespace cura
