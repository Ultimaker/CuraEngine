// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_POLYGONS_POINT_INDEX_H
#define UTILS_POLYGONS_POINT_INDEX_H

#include <vector>

#include "geometry/Point2LL.h"
#include "geometry/Polygon.h"
#include "geometry/Shape.h"


namespace cura
{

/*!
 * A class for iterating over the points in one of the polygons in a \ref Polygons object
 */
template<typename Paths>
class PathsPointIndex
{
public:
    /*!
     * The polygons into which this index is indexing.
     */
    const Paths* polygons_; // (pointer to const polygons)
    size_t poly_idx_; //!< The index of the polygon in \ref PolygonsPointIndex::polygons
    size_t point_idx_; //!< The index of the point in the polygon in \ref PolygonsPointIndex::polygons

    /*!
     * Constructs an empty point index to no polygon.
     *
     * This is used as a placeholder for when there is a zero-construction
     * needed. Since the `polygons` field is const you can't ever make this
     * initialisation useful.
     */
    PathsPointIndex()
        : polygons_(nullptr)
        , poly_idx_(0)
        , point_idx_(0)
    {
    }

    /*!
     * Constructs a new point index to a vertex of a polygon.
     * \param polygons The Polygons instance to which this index points.
     * \param poly_idx The index of the sub-polygon to point to.
     * \param point_idx The index of the vertex in the sub-polygon.
     */
    PathsPointIndex(const Paths* polygons, size_t poly_idx, size_t point_idx)
        : polygons_(polygons)
        , poly_idx_(poly_idx)
        , point_idx_(point_idx)
    {
    }

    /*!
     * Copy constructor to copy these indices.
     */
    PathsPointIndex(const PathsPointIndex& original) = default;

    Point2LL p() const
    {
        if (! polygons_)
        {
            return Point2LL(0, 0);
        }
        return make_point((*polygons_)[poly_idx_][point_idx_]);
    }

    /*!
     * \brief Returns whether this point is initialised.
     */
    bool initialized() const
    {
        return polygons_;
    }

    /*!
     * Get the polygon to which this PolygonsPointIndex refers
     */
    const Polygon& getPolygon() const;

    /*!
     * Test whether two iterators refer to the same polygon in the same polygon list.
     *
     * \param other The PolygonsPointIndex to test for equality
     * \return Wether the right argument refers to the same polygon in the same ListPolygon as the left argument.
     */
    bool operator==(const PathsPointIndex& other) const
    {
        return polygons_ == other.polygons_ && poly_idx_ == other.poly_idx_ && point_idx_ == other.point_idx_;
    }

    bool operator!=(const PathsPointIndex& other) const
    {
        return ! (*this == other);
    }

    bool operator<(const PathsPointIndex& other) const
    {
        return this->p() < other.p();
    }

    PathsPointIndex& operator=(const PathsPointIndex& other)
    {
        polygons_ = other.polygons_;
        poly_idx_ = other.poly_idx_;
        point_idx_ = other.point_idx_;
        return *this;
    }

    //! move the iterator forward (and wrap around at the end)
    PathsPointIndex& operator++()
    {
        point_idx_ = (point_idx_ + 1) % (*polygons_)[poly_idx_].size();
        return *this;
    }

    //! move the iterator backward (and wrap around at the beginning)
    PathsPointIndex& operator--()
    {
        if (point_idx_ == 0)
        {
            point_idx_ = (*polygons_)[poly_idx_].size();
        }
        point_idx_--;
        return *this;
    }

    //! move the iterator forward (and wrap around at the end)
    PathsPointIndex next() const
    {
        PathsPointIndex ret(*this);
        ++ret;
        return ret;
    }

    //! move the iterator backward (and wrap around at the beginning)
    PathsPointIndex prev() const
    {
        PathsPointIndex ret(*this);
        --ret;
        return ret;
    }
};

using PolygonsPointIndex = PathsPointIndex<Shape>;


/*!
 * Locator to extract a line segment out of a \ref PolygonsPointIndex
 */
struct PolygonsPointIndexSegmentLocator
{
    std::pair<Point2LL, Point2LL> operator()(const PolygonsPointIndex& val) const;
};


/*!
 * Locator of a \ref PolygonsPointIndex
 */
template<typename Paths>
struct PathsPointIndexLocator
{
    Point2LL operator()(const PathsPointIndex<Paths>& val) const
    {
        return make_point(val.p());
    }
};

using PolygonsPointIndexLocator = PathsPointIndexLocator<Shape>;

} // namespace cura

namespace std
{
/*!
 * Hash function for \ref PolygonsPointIndex
 */
template<>
struct hash<cura::PolygonsPointIndex>
{
    size_t operator()(const cura::PolygonsPointIndex& lpi) const
    {
        return std::hash<cura::Point2LL>()(lpi.p());
    }
};
} // namespace std


#endif // UTILS_POLYGONS_POINT_INDEX_H
