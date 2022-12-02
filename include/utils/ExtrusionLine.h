//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#ifndef UTILS_EXTRUSION_LINE_H
#define UTILS_EXTRUSION_LINE_H

#include "ExtrusionJunction.h"
#include "polygon.h"

namespace cura
{

/*!
 * Represents a polyline (not just a line) that is to be extruded with variable
 * line width.
 *
 * This polyline is a sequence of \ref ExtrusionJunction, with a bit of metadata
 * about which inset it represents.
 */
struct ExtrusionLine
{
    /*!
     * Which inset this path represents, counted from the outside inwards.
     *
     * The outer wall has index 0.
     */
    size_t inset_idx;

    /*!
     * If a thin piece needs to be printed with an odd number of walls (e.g. 5
     * walls) then there will be one wall in the middle that is not a loop. This
     * field indicates whether this path is such a line through the middle, that
     * has no companion line going back on the other side and is not a closed
     * loop.
     */
    bool is_odd;

    /*!
     * Whether this is a closed polygonal path
     */
    bool is_closed;

    /*!
     * Gets the number of vertices in this polygon.
     * \return The number of vertices in this polygon.
     */
    size_t size() const
    {
        return junctions.size();
    }

    /*!
     * Whether there are no junctions.
     */
    bool empty() const
    {
        return junctions.empty();
    }

    /*!
     * The list of vertices along which this path runs.
     *
     * Each junction has a width, making this path a variable-width path.
     */
    std::vector<ExtrusionJunction> junctions;

    ExtrusionLine(const size_t inset_idx, const bool is_odd);

    ExtrusionLine()
    : inset_idx(-1)
    , is_odd(true)
    , is_closed(false)
    {}

    ExtrusionLine(const ExtrusionLine& other)
    : inset_idx(other.inset_idx)
    , is_odd(other.is_odd)
    , is_closed(other.is_closed)
    , junctions(other.junctions)
    {}
    
    ExtrusionLine& operator=(ExtrusionLine&& other)
    {
        junctions = std::move(other.junctions);
        inset_idx = other.inset_idx;
        is_odd = other.is_odd;
        is_closed = other.is_closed;
        return *this;
    }

    ExtrusionLine& operator=(const ExtrusionLine& other)
    {
        junctions = other.junctions;
        inset_idx = other.inset_idx;
        is_odd = other.is_odd;
        is_closed = other.is_closed;
        return *this;
    }

    
    std::vector<ExtrusionJunction>::const_iterator begin() const
    {
        return junctions.begin();
    }

    std::vector<ExtrusionJunction>::const_iterator end() const
    {
        return junctions.end();
    }

    std::vector<ExtrusionJunction>::const_reverse_iterator rbegin() const
    {
        return junctions.rbegin();
    }

    std::vector<ExtrusionJunction>::const_reverse_iterator rend() const
    {
        return junctions.rend();
    }

    std::vector<ExtrusionJunction>::const_reference front() const
    {
        return junctions.front();
    }

    std::vector<ExtrusionJunction>::const_reference back() const
    {
        return junctions.back();
    }

    const ExtrusionJunction& operator[] (unsigned int index) const
    {
        return junctions[index];
    }

    ExtrusionJunction& operator[] (unsigned int index)
    {
        return junctions[index];
    }

    std::vector<ExtrusionJunction>::iterator begin()
    {
        return junctions.begin();
    }

    std::vector<ExtrusionJunction>::iterator end()
    {
        return junctions.end();
    }

    std::vector<ExtrusionJunction>::reference front()
    {
        return junctions.front();
    }

    std::vector<ExtrusionJunction>::reference back()
    {
        return junctions.back();
    }

    template <typename... Args>
    void emplace_back(Args&&... args)
    {
        junctions.emplace_back(args...);
    }

    void remove(unsigned int index)
    {
        junctions.erase(junctions.begin() + index);
    }

    void insert(size_t index, const ExtrusionJunction& p)
    {
        junctions.insert(junctions.begin() + index, p);
    }

    template <class iterator>
    std::vector<ExtrusionJunction>::iterator insert(std::vector<ExtrusionJunction>::const_iterator pos, iterator first, iterator last)
    {
        return junctions.insert(pos, first, last);
    }

    void clear()
    {
        junctions.clear();
    }

    void reverse()
    {
        std::reverse(junctions.begin(), junctions.end());
    }
    
    /*!
     * Sum the total length of this path.
     */
    coord_t getLength() const;
    coord_t polylineLength() const { return getLength(); }

    /*!
     * Put all junction locations into a polygon object.
     * 
     * When this path is not closed the returned Polygon should be handled as a polyline, rather than a polygon.
     */
    Polygon toPolygon() const
    {
        Polygon ret;
        
        for (const ExtrusionJunction& j : junctions)
            ret.add(j.p);
        
        return ret;
    }

    /*!
     * Get the minimal width of this path
     */
    coord_t getMinimalWidth() const;
};

using VariableWidthLines = std::vector<ExtrusionLine>; //<! The ExtrusionLines generated by libArachne
} // namespace cura
#endif // UTILS_EXTRUSION_LINE_H
