// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_OPEN_POLYLINE_H
#define GEOMETRY_OPEN_POLYLINE_H

#include "geometry/Polyline.h"

namespace cura
{

/*!
 *  @brief Represents a polyline that is explicitely not closed
 *  @sa https://github.com/Ultimaker/CuraEngine/wiki/Geometric-Base-Types#openpolyline
 *  @note Open polylines are sometimes used to represent actually closed polylines. In this case
 *        the first and last point are at the very same position. This should not be done, but
 *        it exists all around the engine for historical reasons. The behavior is however deprecated
 *        and should not be used in the future
 */
class OpenPolyline : public Polyline
{
public:
    /*! @brief Builds an empty polyline */
    OpenPolyline() = default;

    /*!
     * \brief Creates a copy of the given polyline
     * \warning A copy of the points list is made, so this constructor is somehow "slow"
     */
    OpenPolyline(const OpenPolyline& other) = default;

    /*!
     * \brief Constructor that takes ownership of the inner points list from the given polyline
     * \warning This constructor is fast because it does not allocate data, but it will clear
     *          the source object
     */
    OpenPolyline(OpenPolyline&& other) = default;

    /*!
     * \brief Constructor with a points initializer list, provided for convenience
     * \warning A copy of the points list is made, so this constructor is somehow "slow"
     */
    OpenPolyline(const std::initializer_list<Point2LL>& initializer)
        : Polyline{ initializer }
    {
    }

    /*!
     * \brief Constructor with an existing list of points
     * \warning A copy of the points list is made, so this constructor is somehow "slow"
     */
    explicit OpenPolyline(const ClipperLib::Path& points)
        : Polyline{ points }
    {
    }

    /*!
     * \brief Constructor that takes ownership of the given list of points
     * \warning This constructor is fast because it does not allocate data, but it will clear
     *          the source object
     */
    explicit OpenPolyline(ClipperLib::Path&& points)
        : Polyline{ std::move(points) }
    {
    }

    ~OpenPolyline() override = default;

    /*! @see Polyline::hasClosingSegment() */
    [[nodiscard]] bool hasClosingSegment() const override
    {
        return false; // Definitely not
    }

    /*! @see Polyline::segmentsCount() */
    [[nodiscard]] size_t segmentsCount() const override
    {
        return size() > 1 ? size() - 1 : 0;
    }

    /*! @see Polyline::isValid() */
    [[nodiscard]] bool isValid() const override
    {
        return size() >= 2;
    }

    OpenPolyline& operator=(OpenPolyline&& other) noexcept = default;

    OpenPolyline& operator=(const OpenPolyline& other) = default;
};

} // namespace cura

#endif // GEOMETRY_OPEN_POLYLINE_H
