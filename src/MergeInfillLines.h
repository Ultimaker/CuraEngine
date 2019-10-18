//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MERGE_INFILL_LINES_H
#define MERGE_INFILL_LINES_H

#ifdef BUILD_TESTS
    #include <gtest/gtest_prod.h>
#endif //BUILD_TESTS

#include "utils/IntPoint.h" //For Point.

namespace cura
{

class ExtruderPlan;
class GCodePath;

class MergeInfillLines 
{
#ifdef BUILD_TESTS
    FRIEND_TEST(MergeInfillLinesTest, CalcPathLengthEmpty);
    FRIEND_TEST(MergeInfillLinesTest, CalcPathLengthSingle);
    FRIEND_TEST(MergeInfillLinesTest, CalcPathLengthMultiple);
#endif //BUILD_TESTS

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
    bool mergeInfillLines(std::vector<GCodePath>& paths, const Point& starting_position) const;

private:
    /*
     * The extruder plan that contains the path configurations and settings that
     * we can use.
     */
    ExtruderPlan& extruder_plan;

    /*
     * The nozzle size that's printing these lines.
     */
    const coord_t nozzle_size;

    /*
     * The allowed deviation in the line positioning.
     */
    const coord_t maximum_deviation;

    /*
     *
     */
    coord_t calcPathLength(const Point path_start, GCodePath& path) const;

    /*
     * Use to merge adjacent lines by drawing a line through them. Apply alternative strategy if first line is already merged.
     * Merging is only permitted if both lines have 2 points each, or if the first line is already a merged line and the second is a 2 point line.
     * \param first_is_already_merged Indicate if the first_path is already a merged path.
     * \param first_path The first of the two paths to merge.
     * \param first_path_start The starting point of the first path. Changed if first_path_start_changed is set to true.
     * \param second_path The other of the two paths to merge.
     * \param second_path_start The starting point of the second path.
     * \param first_path_start_changed is a result. If set to true, we have changed first_path_start
     * \return true: success, false: fail because of flow constraints
     */
    bool mergeLinesSideBySide(const bool first_is_already_merged, GCodePath& first_path, const Point first_path_start, GCodePath& second_path, const Point second_path_start, Point& new_first_path_start, coord_t& error_area) const;

    /*
     * Try to merge lines.
     * If success, this changes the destination of the first path and adds the extrusion of
     * the two lines together.
     * The second path is untouched. We must skip that path later.
     * \param first_is_already_merged Indicate if the first_path is already a merged path.
     * \param first_path The first of the two paths to merge.
     * \param first_path_start The starting point of the first path. Changed if first_path_start_changed is set to true.
     * \param second_path The other of the two paths to merge.
     * \param second_path_start The starting point of the second path.
     * \param new_first_path_start if first_path_start is changed, this is the result of that
     * \return true: merged second_path into first_path, second_path can be discarded, false: nothing happened
     */
    bool tryMerge(const bool first_is_already_merged, GCodePath& first_path, const Point first_path_start, GCodePath& second_path, const Point second_path_start, Point& new_first_path_start, coord_t& error_area) const;
};

} //namespace cura

#endif //MERGE_INFILL_LINES_H