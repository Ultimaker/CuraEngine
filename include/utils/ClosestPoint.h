// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "geometry/Polygon.h"

namespace curaengine::utils
{

/*!
 * Result of finding the closest point to a given within a set of polygons, with extra information on where the point is.
 */
template<class LineType>
struct ClosestPoint
{
    ::curaengine::geometry::Point2LL location_; //!< Result location
    const LineType* poly_{ nullptr }; //!< Line in which the result was found (or nullptr if no result was found)
    size_t poly_idx_; //!< The index of the polygon in some Polygons where ClosestPoint::poly can be found
    size_t point_idx_; //!< Index to the first point in the polygon of the line segment on which the result was found

    ClosestPoint(::curaengine::geometry::Point2LL p, size_t pos, const LineType* poly)
        : location_(p)
        , poly_(poly)
        , poly_idx_(NO_INDEX)
        , point_idx_(pos)
    {
    }

    ClosestPoint(::curaengine::geometry::Point2LL p, size_t pos, const LineType* poly, size_t poly_idx)
        : location_(p)
        , poly_(poly)
        , poly_idx_(poly_idx)
        , point_idx_(pos)
    {
    }

    ClosestPoint(const LineType* poly)
        : poly_(poly)
        , poly_idx_(NO_INDEX)
        , point_idx_(NO_INDEX)
    {
    }

    ClosestPoint()
        : poly_idx_(NO_INDEX)
        , point_idx_(NO_INDEX)
    {
    }

    ::curaengine::geometry::Point2LL p() const
    { // conformity with other classes
        return location_;
    }

    bool isValid() const
    {
        return point_idx_ != NO_INDEX;
    }

    bool operator==(const ClosestPoint& rhs) const
    {
        // no need to compare on poy_idx
        // it's sometimes unused while poly is always initialized
        return poly_ == rhs.poly_ && point_idx_ == rhs.point_idx_ && location_ == rhs.location_;
    }
};

using ClosestPointPolygon = ClosestPoint<::curaengine::geometry::Polygon>;

} // namespace curaengine::utils

namespace std
{

template<>
struct hash<::curaengine::utils::ClosestPointPolygon>
{
    size_t operator()(const ::curaengine::utils::ClosestPointPolygon& cpp) const
    {
        return std::hash<::curaengine::geometry::Point2LL>()(cpp.p());
    }
};

} // namespace std
