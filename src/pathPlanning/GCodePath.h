/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef PATH_PLANNING_G_CODE_PATH_H
#define PATH_PLANNING_G_CODE_PATH_H

#include "../SpaceFillType.h"
#include "../GCodePathConfig.h"

#include "TimeMaterialEstimates.h"

namespace cura
{

/*!
 * A class for representing a planned path.
 *
 * A path consists of several segments of the same type of movement: retracted travel, infill extrusion, etc.
 *
 * This is a compact premature representation in which are line segments have the same config, i.e. the config of this path.
 *
 * In the final representation (gcode) each line segment may have different properties,
 * which are added when the generated GCodePaths are processed.
 */
class GCodePath
{
public:
    const GCodePathConfig* config; //!< The configuration settings of the path.
    SpaceFillType space_fill_type; //!< The type of space filling of which this path is a part
    float flow; //!< A type-independent flow configuration (used for wall overlap compensation)
    bool retract; //!< Whether the path is a move path preceded by a retraction move; whether the path is a retracted move path.
    bool perform_z_hop; //!< Whether to perform a z_hop in this path, which is assumed to be a travel path.
    bool perform_prime; //!< Whether this path is preceded by a prime (poop)
    std::vector<Point> points; //!< The points constituting this path.
    bool done;//!< Path is finished, no more moves should be added, and a new path should be started instead of any appending done to this one.

    bool spiralize; //!< Whether to gradually increment the z position during the printing of this path. A sequence of spiralized paths should start at the given layer height and end in one layer higher.

    TimeMaterialEstimates estimates; //!< Naive time and material estimates

    /*!
     * Whether this config is the config of a travel path.
     *
     * \return Whether this config is the config of a travel path.
     */
    bool isTravelPath();

    /*!
     * Get the material flow in mm^3 per mm traversed.
     *
     * \warning Can only be called after the layer height has been set (which is done while writing the gcode!)
     *
     * \return The flow
     */
    double getExtrusionMM3perMM();

    /*!
     * Get the actual line width (modulated by the flow)
     * \return the actual line width as shown in layer view
     */
    int getLineWidth();
};

}//namespace cura

#endif//PATH_PLANNING_G_CODE_PATH_H
