//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef ZSEAMCONFIG_H
#define ZSEAMCONFIG_H

#include "EnumSettings.h" //For the seam type and corner preference settings.
#include "../utils/IntPoint.h" //For Point.

namespace cura
{

/*!
 * Helper class that encapsulates various criteria that define the location of
 * the seam.
 *
 * Instances of this are passed to the order optimizer to specify where the seam
 * is to be placed.
 */
struct ZSeamConfig
{
    /*!
     * Strategy to place the seam (user-specified, shortest distance, sharpest
     * corner, etc.)
     */
    EZSeamType type;

    /*!
     * When using a user-specified position for the seam, this is the position
     * that the user specified.
     */
    Point pos;

    /*!
     * Corner preference type, applicable to various strategies to filter on
     * which corners the seam is allowed to be located.
     */
    EZSeamCornerPrefType corner_pref;

    /*!
     * Default constructor for use when memory must be allocated before it gets
     * filled (like with some data structures).
     *
     * This will select the "shortest" seam strategy.
     */
    ZSeamConfig();

    /*!
     * Create a seam configuration with a custom configuration.
     * \param type The strategy to place the seam.
     * \param pos The position of a user-specified seam.
     * \param corner_pref The corner preference, applicable to some strategies.
     */
    ZSeamConfig(const EZSeamType type, const Point pos, const EZSeamCornerPrefType corner_pref);
};

}

#endif //ZSEAMCONFIG_H