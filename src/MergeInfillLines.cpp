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
        1a. Check if two adjacent lines can be merged.
        1b. If they can, merge both lines into the first line.
        1c. If they are merged, check next that the first line can be merged with
            the line after the second line.
        2. Do a second iteration over all paths to remove the tombstones. */
    bool something_changed = false;

    //For each two adjacent lines, see if they can be merged.
    size_t first_path_index = 0;
    size_t second_path_index = 1;
    for (; second_path_index < paths.size(); second_path_index++)
    {
        GCodePath& first_path = paths[first_path_index];
        GCodePath& second_path = paths[second_path_index];

        if (isConvertible(first_path, second_path))
        {
            /* If we combine two lines, the second path is inside the first
            line, so the iteration after that we need to merge the first line
            with the line after the second line, so we do NOT update
            first_path_index. */
            mergeLines(first_path, second_path);
            something_changed = true;
        }
        else
        {
            /* If we do not combine, the next iteration we must simply merge the
            second path with the line after it. */
            first_path_index = second_path_index;
        }
    }
    return something_changed;
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
