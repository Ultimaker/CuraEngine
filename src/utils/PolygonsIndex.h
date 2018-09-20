/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_POLYGONS_INDEX_H
#define UTILS_POLYGONS_INDEX_H

#include <vector>

#include "intpoint.h"
#include "polygon.h"


namespace cura 
{

/*!
 * A class for iterating over the points in one of the polygons in a \ref Polygons object
 */
class PolygonsIndex
{
public:
    /*!
     * The polygons into which this index is indexing
     */
    const Polygons* polygons; // (pointer to const polygons)
    unsigned int poly_idx; //!< The index of the polygon in \ref PolygonsIndex::polygons

    PolygonsIndex()
    : polygons(nullptr)
    , poly_idx(0)
    {
    }

    PolygonsIndex(const Polygons* polygons, unsigned int poly_idx)
    : polygons(polygons)
    , poly_idx(poly_idx)
    {
    }

    /*!
     * \brief Returns whether this point is initialised.
     */
    bool initialized() const;

    /*!
     * Get the polygon to which this PolygonsIndex refers
     * 
     * \warning Polygon is assumed to be initialized when this function is called
     * 
     * \see \ref PolygonsIndex::initialized
     */
    ConstPolygonRef operator*() const
    {
        assert(polygons && "Polygon is assumed to be initialized when this function is called");
        return (*polygons)[poly_idx];
    }

    /*!
     * Test whether two iterators refer to the same polygon in the same polygon list.
     * 
     * \param other The PolygonsIndex to test for equality
     * \return Wether the right argument refers to the same polygon in the same ListPolygon as the left argument.
     */
    bool operator==(const PolygonsIndex& other) const
    {
        return polygons == other.polygons && poly_idx == other.poly_idx;
    }
    bool operator!=(const PolygonsIndex& other) const
    {
        return !(*this == other);
    }
    PolygonsIndex& operator=(const PolygonsIndex& other)
    {
        polygons = other.polygons;
        poly_idx = other.poly_idx;
        return *this;
    }
};


}//namespace cura


#endif//UTILS_POLYGONS_INDEX_H
