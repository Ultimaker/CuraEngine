// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_WALLTOOLPATHS_H
#define CURAENGINE_WALLTOOLPATHS_H

#include <memory>

#include "BeadingStrategy/BeadingStrategyFactory.h"
#include "settings/Settings.h"
#include "utils/ExtrusionLine.h"
#include "utils/polygon.h"
#include "utils/section_type.h"

namespace cura
{
class WallToolPaths
{
public:
    /*!
     * A class that creates the toolpaths given an outline, nominal bead width and maximum amount of walls
     * \param outline An outline of the area in which the ToolPaths are to be generated
     * \param nominal_bead_width The nominal bead width used in the generation of the toolpaths
     * \param inset_count The maximum number of parallel extrusion lines that make up the wall
     * \param wall_0_inset How far to inset the outer wall, to make it adhere better to other walls.
     * \param settings The settings as provided by the user
     */
    WallToolPaths(const Polygons& outline, const coord_t nominal_bead_width, const size_t inset_count, const coord_t wall_0_inset, const Settings& settings, const int layer_idx, SectionType section_type);

    /*!
     * A class that creates the toolpaths given an outline, nominal bead width and maximum amount of walls
     * \param outline An outline of the area in which the ToolPaths are to be generated
     * \param bead_width_0 The bead width of the first wall used in the generation of the toolpaths
     * \param bead_width_x The bead width of the inner walls used in the generation of the toolpaths
     * \param inset_count The maximum number of parallel extrusion lines that make up the wall
     * \param wall_0_inset How far to inset the outer wall, to make it adhere better to other walls.
     * \param settings The settings as provided by the user
     */
    WallToolPaths(const Polygons& outline, const coord_t bead_width_0, const coord_t bead_width_x, const size_t inset_count, const coord_t wall_0_inset, const Settings& settings, const int layer_idx, SectionType section_type);

    /*!
     * Generates the Toolpaths
     * \return A reference to the newly created ToolPaths. Binned by inset_idx.
     */
    const std::vector<VariableWidthLines>& generate();

    /*!
     * Gets the toolpaths, if this called before \p generate() it will first generate the Toolpaths
     * \return a reference to the toolpaths. Binned by inset_idx.
     */
    const std::vector<VariableWidthLines>& getToolPaths();

    /*!
     * Alternate 'get', for when the vector that'll be inserted in already exists.
     * \param paths The already existing (or empty) paths these new toolpaths are pushed into. Binned by inset_idx.
     */
    void pushToolPaths(std::vector<VariableWidthLines>& paths);

    /*!
     * Compute the inner contour of the walls. This contour indicates where the walled area ends and its infill begins.
     * The inside can then be filled, e.g. with skin/infill for the walls of a part, or with a pattern in the case of
     * infill with extra infill walls.
     */
    void separateOutInnerContour();

    /*!
     * Gets the inner contour of the area which is inside of the generated tool
     * paths.
     *
     * If the walls haven't been generated yet, this will lazily call the
     * \p generate() function to generate the walls with variable width.
     * The resulting polygon will snugly match the inside of the variable-width
     * walls where the walls get limited by the LimitedBeadingStrategy to a
     * maximum wall count.
     * If there are no walls, the outline will be returned.
     * \return The inner contour of the generated walls.
     */
    const Polygons& getInnerContour();

    /*!
     * Removes empty paths from the toolpaths
     * \param toolpaths the toolpaths binned by inset_idx generated with \p generate()
     * \return true if there are still paths left. If all toolpaths were removed it returns false
     */
    static bool removeEmptyToolPaths(std::vector<VariableWidthLines>& toolpaths);

protected:
    /*!
     * Stitch the polylines together and form closed polygons.
     * 
     * Works on both toolpaths and inner contours simultaneously.
     * 
     * \param settings The settings as provided by the user
     */
    static void stitchToolPaths(std::vector<VariableWidthLines>& toolpaths, const Settings& settings);

    /*!
     * Remove polylines shorter than half the smallest line width along that polyline.
     */
    static void removeSmallLines(std::vector<VariableWidthLines>& toolpaths);

    /*!
     * Simplifies the variable-width toolpaths by calling the simplify on every line in the toolpath using the provided
     * settings.
     * \param settings The settings as provided by the user
     */
    static void simplifyToolPaths(std::vector<VariableWidthLines>& toolpaths, const Settings& settings);

private:
    const Polygons& outline; //<! A reference to the outline polygon that is the designated area
    coord_t bead_width_0; //<! The nominal or first extrusion line width with which libArachne generates its walls
    coord_t bead_width_x; //<! The subsequently extrusion line width with which libArachne generates its walls if WallToolPaths was called with the nominal_bead_width Constructor this is the same as bead_width_0
    size_t inset_count; //<! The maximum number of walls to generate
    coord_t wall_0_inset; //<! How far to inset the outer wall. Should only be applied when printing the actual walls, not extra infill/skin/support walls.
    bool print_thin_walls; //<! Whether to enable the widening beading meta-strategy for thin features
    coord_t min_feature_size; //<! The minimum size of the features that can be widened by the widening beading meta-strategy. Features thinner than that will not be printed
    coord_t min_bead_width;  //<! The minimum bead size to use when widening thin model features with the widening beading meta-strategy
    double small_area_length; //<! The length of the small features which are to be filtered out, this is squared into a surface
    coord_t transition_length; //<! The transitioning length when the amount of extrusion lines changes
    bool toolpaths_generated; //<! Are the toolpaths generated
    std::vector<VariableWidthLines> toolpaths; //<! The generated toolpaths binned by inset_idx.
    Polygons inner_contour;  //<! The inner contour of the generated toolpaths
    const Settings& settings;
    int layer_idx;
    SectionType section_type;
};
} // namespace cura

#endif // CURAENGINE_WALLTOOLPATHS_H
