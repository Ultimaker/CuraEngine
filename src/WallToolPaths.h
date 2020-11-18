// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef CURAENGINE_WALLTOOLPATHS_H
#define CURAENGINE_WALLTOOLPATHS_H

#include <memory>

#include "BeadingStrategy/BeadingStrategyFactory.h"
#include "settings/Settings.h"
#include "utils/ExtrusionLine.h"
#include "utils/polygon.h"

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
     * \param settings The settings as provided by the user
     */
    WallToolPaths(const Polygons& outline, coord_t nominal_bead_width, coord_t inset_count, const Settings& settings);

    /*!
     * A class that creates the toolpaths given an outline, nominal bead width and maximum amount of walls
     * \param outline An outline of the area in which the ToolPaths are to be generated
     * \param bead_width_0 The bead width of the first wall used in the generation of the toolpaths
     * \param bead_width_x The bead width of the inner walls used in the generation of the toolpaths
     * \param inset_count The maximum number of parallel extrusion lines that make up the wall
     * \param settings The settings as provided by the user
     */
    WallToolPaths(const Polygons& outline, coord_t bead_width_0, coord_t bead_width_x, coord_t inset_count, const Settings& settings);

    /*!
     * Generates the Toolpaths
     * \return A reference to the newly create  ToolPaths
     */
    const VariableWidthPaths& generate();

    /*!
     * Gets the toolpaths, if this called before \p generate() it will first generate the Toolpaths
     * \return a reference to the toolpaths
     */
    const VariableWidthPaths& getToolPaths();

    /*!
     * Gets the inner contour of the area which is inside of the generated ToolPaths. This is for now a simple offset
     * of the outline. But after the implementation of CURA-7681 this will return the actual inside contour.
     * If this is called before \p generate() it will first generate the ToolPaths
     * If this is called when the inset count is 0 it will return a reference to the outline
     * \return A reference to the inner contour
     */
    const Polygons& getInnerContour();

    /*!
     * Gets the outline
     * \return a reference to the outline
     */
    const Polygons& getOutline() const;

    /*!
     * Obtains the inner contour of the generated ToolPaths. Not yet implemented. See CURA-7681
     * \param toolpaths the toolpaths used to determine the inner contour
     * \return
     */
    static Polygons innerContourFromToolpaths(const VariableWidthPaths& toolpaths);

    /*!
     * Removes empty paths from the toolpaths
     * \param toolpaths the VariableWidthPaths generated with \p generate()
     * \return true if there are still paths left. If all toolpaths were removed it returns false
     */
    static bool removeEmptyToolPaths(VariableWidthPaths& toolpaths);

    /*!
     * Simplifies the generated toolpaths
     * \return
     */
    void simplifyToolpaths();

private:
    const Polygons& outline; //<! A reference to the outline polygon that is the designated area
    const coord_t bead_width_0; //<! The nominal or first extrusion line width with which libArachne generates its walls
    const coord_t bead_width_x; //<! The subsequently extrusion line width with which libArachne generates its walls if WallToolPaths was called with the nominal_bead_width Constructor this is the same as bead_width_0
    const coord_t inset_count; //<! The maximum number of walls to generate
    const StrategyType strategy_type; //<! The wall generating strategy
    const bool print_thin_walls; //<! Whether to enable the widening beading meta-strategy for thin features
    std::unique_ptr<coord_t> min_feature_size; //<! The minimum size of the features that can be widened by the widening beading meta-strategy. Features thinner than that will not be printed
    std::unique_ptr<coord_t> min_bead_width;  //<! The minimum bead size to use when widening thin model features with the widening beading meta-strategy
    const double small_area_length; //<! The length of the small features which are to be filtered out, this is squared into a surface
    const coord_t transition_length; //<! The transitioning length when the amount of extrusion lines changes
    bool toolpaths_generated; //<! Are the toolpaths generated
    VariableWidthPaths toolpaths; //<! The generated toolpaths
    Polygons inner_contour;  //<! The inner contour of the generated toolpaths
    const Settings& settings;
};
} // namespace cura

#endif // CURAENGINE_WALLTOOLPATHS_H
