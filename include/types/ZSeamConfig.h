// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "types/EZSeamCornerPrefType.h"
#include "types/EZSeamType.h"
#include "types/geometry/Point2LL.h"

namespace curaengine::types
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
    EZSeamType type_;

    /*!
     * When using a user-specified position for the seam, this is the position
     * that the user specified.
     */
    types::geometry::Point2LL pos_;

    /*!
     * Corner preference type, if using the sharpest corner strategy.
     */
    EZSeamCornerPrefType corner_pref_;

    /*!
     * Prevent 'smoothed out' corners (corners that are spread over multiple, very close together vertices),
     * by simplifying the polygon that the corners are detected on by this ammount.
     * This does _not_ influence the path, the simplified polygon is a temporary constructed within the algorithm.
     */
    types::coord_t simplify_curvature_;

    /*!
     * Create a seam configuration with a custom configuration.
     * \param type The strategy to place the seam.
     * \param pos The position of a user-specified seam.
     * \param corner_pref The corner preference, when using the sharpest corner strategy.
     * \param by how much to simplify the curvature (when detecting corners), as otherwise 'smooth' corners are penalized.
     */
    ZSeamConfig(
        const EZSeamType type = EZSeamType::Shortest,
        const types::geometry::Point2LL pos = types::geometry::Point2LL(0, 0),
        const EZSeamCornerPrefType corner_pref = EZSeamCornerPrefType::ZSeamCornerPrefInner,
        const types::coord_t simplify_curvature = 0);
};

} // namespace curaengine::types
