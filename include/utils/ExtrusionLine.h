// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.


#ifndef UTILS_EXTRUSION_LINE_H
#define UTILS_EXTRUSION_LINE_H

#include <algorithm>

#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/reverse.hpp>
#include <range/v3/view/sliding.hpp>

#include "ExtrusionJunction.h"
#include "geometry/Polygon.h"
#include "geometry/Shape.h"

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
    size_t inset_idx_;

    /*!
     * If a thin piece needs to be printed with an odd number of walls (e.g. 5
     * walls) then there will be one wall in the middle that is not a loop. This
     * field indicates whether this path is such a line through the middle, that
     * has no companion line going back on the other side and is not a closed
     * loop.
     */
    bool is_odd_;

    /*!
     * Whether this is a closed polygonal path
     */
    bool is_closed_;

    /*!
     * The list of vertices along which this path runs.
     *
     * Each junction has a width, making this path a variable-width path.
     */
    std::vector<ExtrusionJunction> junctions_;

    /*!
     * Gets the number of vertices in this polygon.
     * \return The number of vertices in this polygon.
     */
    size_t size() const
    {
        return junctions_.size();
    }

    /*!
     * Gets the vertex at the given index.
     * \param idx The index of the vertex to get.
     * \return The vertex at the given index.
     */
    bool is_outer_wall() const
    {
        return inset_idx_ == 0;
    }

    /*!
     * Whether there are no junctions.
     */
    bool empty() const
    {
        return junctions_.empty();
    }

    ExtrusionLine(size_t inset_idx = std::numeric_limits<size_t>::max(), bool is_odd = false, bool is_closed = false)
        : inset_idx_(inset_idx)
        , is_odd_(is_odd)
        , is_closed_(is_closed)
    {
    }

    ExtrusionLine(const ExtrusionLine& other)
        : inset_idx_(other.inset_idx_)
        , is_odd_(other.is_odd_)
        , is_closed_(other.is_closed_)
        , junctions_(other.junctions_)
    {
    }

    ExtrusionLine& operator=(ExtrusionLine&& other)
    {
        junctions_ = std::move(other.junctions_);
        inset_idx_ = other.inset_idx_;
        is_odd_ = other.is_odd_;
        is_closed_ = other.is_closed_;
        return *this;
    }

    ExtrusionLine& operator=(const ExtrusionLine& other)
    {
        junctions_ = other.junctions_;
        inset_idx_ = other.inset_idx_;
        is_odd_ = other.is_odd_;
        is_closed_ = other.is_closed_;
        return *this;
    }


    std::vector<ExtrusionJunction>::const_iterator begin() const
    {
        return junctions_.begin();
    }

    std::vector<ExtrusionJunction>::const_iterator end() const
    {
        return junctions_.end();
    }

    std::vector<ExtrusionJunction>::const_reverse_iterator rbegin() const
    {
        return junctions_.rbegin();
    }

    std::vector<ExtrusionJunction>::const_reverse_iterator rend() const
    {
        return junctions_.rend();
    }

    std::vector<ExtrusionJunction>::const_reference front() const
    {
        return junctions_.front();
    }

    std::vector<ExtrusionJunction>::const_reference back() const
    {
        return junctions_.back();
    }

    const ExtrusionJunction& operator[](size_t index) const
    {
        return junctions_[index];
    }

    ExtrusionJunction& operator[](size_t index)
    {
        return junctions_[index];
    }

    std::vector<ExtrusionJunction>::iterator begin()
    {
        return junctions_.begin();
    }

    std::vector<ExtrusionJunction>::iterator end()
    {
        return junctions_.end();
    }

    std::vector<ExtrusionJunction>::reference front()
    {
        return junctions_.front();
    }

    std::vector<ExtrusionJunction>::reference back()
    {
        return junctions_.back();
    }

    template<typename... Args>
    void emplace_back(Args&&... args)
    {
        junctions_.emplace_back(args...);
    }

    void remove(unsigned int index)
    {
        junctions_.erase(junctions_.begin() + index);
    }

    void insert(size_t index, const ExtrusionJunction& p)
    {
        junctions_.insert(junctions_.begin() + static_cast<long>(index), p);
    }

    template<class iterator>
    std::vector<ExtrusionJunction>::iterator insert(std::vector<ExtrusionJunction>::const_iterator pos, iterator first, iterator last)
    {
        return junctions_.insert(pos, first, last);
    }

    void clear()
    {
        junctions_.clear();
    }

    void reverse()
    {
        std::reverse(junctions_.begin(), junctions_.end());
    }

    /*!
     * Sum the total length of this path.
     */
    coord_t length() const;

    /*!
     * Put all junction locations into a polygon object.
     *
     * When this path is not closed the returned Polygon should be handled as a polyline, rather than a polygon.
     */
    Polygon toPolygon() const
    {
        Polygon ret;

        for (const ExtrusionJunction& j : junctions_)
            ret.push_back(j.p_);

        return ret;
    }

    /*!
     * Get the minimal width of this path
     */
    coord_t getMinimalWidth() const;

    bool shorterThan(const coord_t check_length) const;
};

using VariableWidthLines = std::vector<ExtrusionLine>; //<! The ExtrusionLines generated by libArachne
} // namespace cura
#endif // UTILS_EXTRUSION_LINE_H
