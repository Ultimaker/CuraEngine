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
     * Generates the Toolpaths
     * \return A reference to the newly create  ToolPaths
     */
    const VariableWidthPath& generate();

    /*!
     * Gets the toolpaths, if this called before \p generate() it will first generate the Toolpaths
     * \return a reference to the toolpaths
     */
    const VariableWidthPath& getToolPaths();

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
    static Polygons innerContourFromToolpaths(const VariableWidthPath& toolpaths);

private:
    const Polygons& outline; //<! A reference to the outline polygon that is the designated area
    const coord_t nominal_bead_width; //<! The nominal extrusion line width with which libArachne generates its walls
    const coord_t inset_count; //<! The maximum number of walls to generate
    const StrategyType strategy_type; //<! The wall generating strategy
    const bool widening_beading_enabled; //<! Is the widening beading setting enabled
    std::unique_ptr<coord_t> min_bead_width;  //<! The minimum bead size to use when the widening beading setting is enabled
    std::unique_ptr<coord_t> min_feature_size; //<! The minimum feature size to us when the widening beading setting is enabled
    const double small_area_length; //<! The length of the small features which are to be filtered out, this is squared into a surface
    const coord_t transition_length; //<! The transitioning length when the amount of extrusion lines changes
    bool toolpaths_generated; //<! Are the toolpaths generated
    VariableWidthPath toolpaths; //<! The generated toolpaths
    Polygons inner_contour;  //<! The inner contour of the generated toolpaths
};
} // namespace cura

#endif // CURAENGINE_WALLTOOLPATHS_H
