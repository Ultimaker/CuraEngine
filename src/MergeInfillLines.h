//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MERGE_INFILL_LINES_H
#define MERGE_INFILL_LINES_H

#include "pathPlanning/GCodePath.h" //To accept and generate g-code paths.

namespace cura
{

class MergeInfillLines 
{
public:
    /*
     * Create a new merger instance.
     * \param plan An extruder plan that contains the path configurations and
     * settings that we can use.
     */
    MergeInfillLines(ExtruderPlan& plan);

    /*
     * Check for lots of small moves and combine them into one large line.
     * \param paths The actual paths that must be merged. These paths may be
     * modified in-place if anything is to be merged.
     * \param starting_position Where the first line starts.
     * \return Whether anything was changed in the paths of the plan.
     */
    bool mergeInfillLines(std::vector<GCodePath>& paths, const Point starting_position) const;

private:
    /*
     * The extruder plan that contains the path configurations and settings that
     * we can use.
     */
    ExtruderPlan& extruder_plan;

    /*
     * Checks if two paths can be merged.
     * \param first_path The first of the two paths to merge.
     * \param first_path_start The starting point of the first path.
     * \param second_path The other of the two paths to merge.
     * \param second_path_start The starting point of the second path.
     * \return ``True`` if the two lines can be merged into one, or ``False`` if
     * they can't.
     */
    //bool isConvertible(const GCodePath& first_path, Point first_path_start, const GCodePath& second_path, const Point second_path_start) const;

    /*
     * Strategy to merge lines
     */
    void mergeLinesSharedEndStartPoint(GCodePath& first_path, const Point first_path_start, GCodePath& second_path, const Point second_path_start) const;

    /*
     * Merge adjacent lines by drawing a line through them. Apply alternative strategy if first line is already merged.
     * Merging is only permitted if both lines have 2 points each, or if the first line is already a merged line and the second is a 2 point line.
     */
    void mergeLinesSideBySide(const bool first_is_already_merged, GCodePath& first_path, const Point first_path_start, GCodePath& second_path, const Point second_path_start) const;

    /*
     * \return true: merged into first_path, second_path can be discarded
     */
    bool tryMerge(const bool first_is_already_merged, GCodePath& first_path, const Point first_path_start, GCodePath& second_path, Point second_path_start) const;

    /*
     * Merges two lines together.
     *
     * This changes the destination of the first path and adds the extrusion of
     * the two lines together.
     * The second path is untouched. We must skip that path later.
     * \param first_path The line to merge the second path into.
     * \param first_path_start Where the first path starts off.
     * \param second_path The line to merge into the first path.
     * \param second_path_start Where the second path starts off.
     */
    //void mergeLines(GCodePath& first_path, const Point first_path_start, GCodePath& second_path, const Point second_path_start, const bool mod_start) const;
};

} //namespace cura

#endif //MERGE_INFILL_LINES_H