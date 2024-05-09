// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_MIXED_LINES_SET_H
#define GEOMETRY_MIXED_LINES_SET_H

#include <memory>

#include "geometry/ClosedLinesSet.h"
#include "geometry/OpenLinesSet.h"
#include "utils/Coord_t.h"

namespace cura
{

class Polyline;
class ClosedPolyline;
class Polygon;
class Shape;

using PolylinePtr = std::shared_ptr<Polyline>;
using OpenPolylinePtr = std::shared_ptr<OpenPolyline>;

/*!
 * \brief Convenience definition for a container that can hold any type of polyline.
 * \sa https://github.com/Ultimaker/CuraEngine/wiki/Geometric-Base-Types#mixedlinesset
 */
class MixedLinesSet : public std::vector<PolylinePtr>
{
public:
    /*!
     * \brief Computes the offset of all the polylines contained in the set. The polylines may
     *        be of different types, and polylines are polygons are treated differently.
     * \param distance The distance to increase the polylines from, or decrase if negative
     * \param join_type The type of tip joint to be used (for open polylines only)
     * \return A shape containing the offsetted polylines. This may contain many unjoined polygons,
     *         but no overlapping ones.
     */
    [[nodiscard]] Shape offset(coord_t distance, ClipperLib::JoinType join_type = ClipperLib::jtMiter, double miter_limit = 1.2) const;

    /*! @brief Adds a copy of the given polyline to the set */
    void push_back(const OpenPolyline& line);

    /*! @brief Adds a copy of the given polyline to the set */
    void push_back(const Polygon& line);

    /*! @brief Adds a copy of the given polyline to the set */
    void push_back(OpenPolyline&& line);

    /*! @brief Adds a copy of the given polyline to the set */
    void push_back(ClosedPolyline&& line);

    /*! @brief Adds the given shared pointer to the set. The pointer reference count will be incremeted but no data is actually copied. */
    void push_back(const OpenPolylinePtr& line);

    /*! @brief Adds the given shared pointer to the set. The pointer reference count will be incremeted but no data is actually copied. */
    void push_back(const PolylinePtr& line);

    /*! @brief Adds a copy of all the polygons contained in the shape */
    void push_back(const Shape& shape);

    /*! @brief Adds a copy of all the polygons contained in the set */
    void push_back(const LinesSet<Polygon>& lines_set);

    /*! @brief Adds a copy of all the polylines contained in the set */
    void push_back(const OpenLinesSet& lines_set);

    /*! @brief Adds a copy of all the polylines contained in the set */
    void push_back(OpenLinesSet&& lines_set);

    /*! @brief Adds a copy of all the polylines contained in the set */
    void push_back(ClosedLinesSet&& lines_set);

    /*! \brief Computes the total lenght of all the polylines in the set */
    [[nodiscard]] coord_t length() const;
};

} // namespace cura

#endif // GEOMETRY_MIXED_LINES_SET_H
