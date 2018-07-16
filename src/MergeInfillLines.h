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
     * \return Whether anything was changed in the paths of the plan.
     */
    bool mergeInfillLines(std::vector<GCodePath>& paths) const;

private:
    /*
     * The extruder plan that contains the path configurations and settings that
     * we can use.
     */
    ExtruderPlan& extruder_plan;

    /*
     * Checks if two paths can be merged.
     * \param first_path The first of the two paths to merge.
     * \param second_path The other of the two paths to merge.
     * \return ``True`` if the two lines can be merged into one, or ``False`` if
     * they can't.
     */
    bool isConvertible(GCodePath& first_path, GCodePath& second_path) const;

    /*
     * Merges two lines together.
     *
     * This changes the destination of the first path and adds the extrusion of
     * the two lines together.
     * The second path is untouched. We must skip that path later.
     * \param first_path The line to merge the second path into.
     * \param second_path The line to merge into the first path.
     */
    void mergeLines(GCodePath& first_path, const GCodePath& second_path) const;
};

} //namespace cura

#endif //MERGE_INFILL_LINES_H