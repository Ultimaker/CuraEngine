//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef ZSEAMCONFIG_H
#define ZSEAMCONFIG_H

#include "EnumSettings.h" //For EZSeamType and EZSeamCornerPrefType.
#include "../utils/IntPoint.h" //To store the preferred seam position.

namespace cura
{

/*!
 * Helper class that encapsulates the various criteria that define the location
 * of the z-seam.
 * Instances of this are passed to the PathOrderOptimizer to specify where the
 * seam is to be located.
 */
struct ZSeamConfig
{
    /*!
     * Strategy to place the seam (user-specified, shortest distance, sharpest
     * corner, etc.).
     */
    EZSeamType type;

    /*!
     * When using a user-specified position for the seam, this is the position
     * that the user specified.
     */
    Point pos;

    /*!
     * Corner preference type, if using the sharpest corner strategy.
     */
    EZSeamCornerPrefType corner_pref;

    /*!
     * Prevent 'smoothed out' corners (corners that are spread over multiple, very close together vertices),
     * by simplifying the polygon that the corners are detected on by this ammount.
     * This does _not_ influence the path, the simplified polygon is a temporary constructed within the algorithm.
     */
    coord_t simplify_curvature;

    /*!
     * Create a seam configuration with a custom configuration.
     * \param type The strategy to place the seam.
     * \param pos The position of a user-specified seam.
     * \param corner_pref The corner preference, when using the sharpest corner strategy.
     * \param by how much to simplify the curvature (when detecting corners), as otherwise 'smooth' corners are penalized.
     */
    ZSeamConfig
    (
        const EZSeamType type = EZSeamType::SHORTEST,
        const Point pos = Point(0, 0),
        const EZSeamCornerPrefType corner_pref = EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE,
        const coord_t simplify_curvature = 0
    );
};

} //Cura namespace.

#endif //ZSEAMCONFIG_H