// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_LIST_POLY_IT_H
#define UTILS_LIST_POLY_IT_H

#include <list>
#include <vector>

#include "geometry/Point2LL.h"


namespace cura
{

class Polygon;
class Shape;

using ListPolygon = std::list<Point2LL>; //!< A polygon represented by a linked list instead of a vector
using ListPolygons = std::vector<ListPolygon>; //!< Polygons represented by a vector of linked lists instead of a vector of vectors

/*!
 * A wrapper class for a ListPolygon::iterator and a reference to the containing ListPolygon
 */
class ListPolyIt
{
public:
    ListPolyIt(const ListPolyIt& other)
        : poly_(other.poly_)
        , it_(other.it_)
    {
    }

    ListPolyIt(ListPolygon& poly, ListPolygon::iterator it)
        : poly_(&poly)
        , it_(it)
    {
    }

    Point2LL& p() const
    {
        return *it_;
    }

    /*!
     * Test whether two iterators refer to the same polygon in the same polygon list.
     *
     * \param other The ListPolyIt to test for equality
     * \return Wether the right argument refers to the same polygon in the same ListPolygon as the left argument.
     */
    bool operator==(const ListPolyIt& other) const
    {
        return poly_ == other.poly_ && it_ == other.it_;
    }

    bool operator!=(const ListPolyIt& other) const
    {
        return ! (*this == other);
    }

    ListPolyIt& operator=(const ListPolyIt& other)
    {
        poly_ = other.poly_;
        it_ = other.it_;
        return *this;
    }

    //! move the iterator forward (and wrap around at the end)
    ListPolyIt& operator++()
    {
        ++it_;
        if (it_ == poly_->end())
        {
            it_ = poly_->begin();
        }
        return *this;
    }

    //! move the iterator backward (and wrap around at the beginning)
    ListPolyIt& operator--()
    {
        if (it_ == poly_->begin())
        {
            it_ = poly_->end();
        }
        --it_;
        return *this;
    }

    //! move the iterator forward (and wrap around at the end)
    ListPolyIt next() const
    {
        ListPolyIt ret(*this);
        ++ret;
        return ret;
    }

    //! move the iterator backward (and wrap around at the beginning)
    ListPolyIt prev() const
    {
        ListPolyIt ret(*this);
        --ret;
        return ret;
    }

    //! Remove this point from the list polygon
    void remove() const
    {
        poly_->erase(it_);
    }

    /*!
     * Convert Polygons to ListPolygons
     *
     * \param polys The polygons to convert
     * \param result The converted polygons
     */
    static void convertPolygonsToLists(const Shape& shape, ListPolygons& result);

    /*!
     * Convert Polygons to ListPolygons
     *
     * \param polys The polygons to convert
     * \param result The converted polygons
     */
    static void convertPolygonToList(const Polygon& poly, ListPolygon& result);

    /*!
     * Convert ListPolygons to Polygons
     *
     * \param list_polygons The polygons to convert
     * \param polygons The converted polygons
     */
    static void convertListPolygonsToPolygons(const ListPolygons& list_polygons, Shape& polygons);

    /*!
     * Convert ListPolygons to Polygons
     *
     * \param list_polygons The polygons to convert
     * \param polygons The converted polygons
     */
    static void convertListPolygonToPolygon(const ListPolygon& list_polygon, Polygon& polygon);

    /*!
     * Insert a point into a ListPolygon if it's not a duplicate of the point before or the point after.
     *
     * \param before Iterator to the point before the point to insert
     * \param after Iterator to the point after the point to insert
     * \param to_insert The point to insert into the ListPolygon in between \p before and \p after
     * \return Iterator to the newly inserted point, or \p before or \p after in case to_insert was already in the polygon
     */
    static ListPolyIt insertPointNonDuplicate(const ListPolyIt before, const ListPolyIt after, const Point2LL to_insert);

private:
    ListPolygon* poly_; //!< The polygon
    ListPolygon::iterator it_; //!< The iterator into ListPolyIt::poly
};


} // namespace cura

namespace std
{
/*!
 * Hash function for \ref ListPolyIt
 */
template<>
struct hash<cura::ListPolyIt>
{
    size_t operator()(const cura::ListPolyIt& lpi) const
    {
        return std::hash<cura::Point2LL>()(lpi.p());
    }
};
} // namespace std


#endif // UTILS_LIST_POLY_IT_H
