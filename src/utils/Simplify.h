//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_SIMPLIFY_H
#define UTILS_SIMPLIFY_H

#include "polygon.h"
#include "ExtrusionLine.h"
#include "../settings/Settings.h" //To load the parameters from a Settings object.

namespace cura
{

/*!
 * Utility class to reduce the resolution of polygons and polylines, under
 * certain constraints.
 *
 * This class implements a modified version of Ramer-Douglas-Peucker which is
 * meant to reduce the resolution of polylines or polygons.
 */
class Simplify
{
public:
    /*!
     * Construct a simplifier, storing the simplification parameters in the
     * instance (as a factory pattern).
     * \param max_resolution Line segments smaller than this are considered for
     * joining with other line segments.
     * \param max_deviation If removing a vertex would cause a deviation larger
     * than this, it cannot be removed.
     * \param max_area_deviation If removing a vertex would cause the covered
     * area in total to change more than this, it cannot be removed.
     */
    Simplify(const coord_t max_resolution, const coord_t max_deviation, const coord_t max_area_deviation);

    /*!
     * Construct a simplifier using the resolution settings inside of the given
     * settings object.
     * \param settings A settings object to obtain the simplification parameters
     * from.
     */
    Simplify(const Settings& settings);

    /*!
     * Simplify a polygon.
     * \param polygon The polygon to simplify.
     */
    Polygon polygon(const PolygonRef polygon);

    /*!
     * Simplify a variable-line-width polygon.
     * \param polygon The polygon to simplify.
     */
    ExtrusionLine polygon(const ExtrusionLine& polygon);

    /*!
     * Simplify a polyline.
     *
     * The endpoints of the polyline cannot be altered.
     * \param polyline The polyline to simplify.
     */
    Polygon polyline(const PolygonRef polyline);

    /*!
     * Simplify a variable-line-width polyline.
     * The endpoints of the polyline cannot be altered.
     * \param polyline The polyline to simplify.
     */
    ExtrusionLine polyline(const ExtrusionLine& polyline);
};

} //namespace cura

#endif //UTILS_SIMPLIFY_H

