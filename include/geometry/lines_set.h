// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_LINES_SET_H
#define GEOMETRY_LINES_SET_H

#include <vector>

#include "geometry/point2ll.h"

namespace cura
{

class Shape;
template<class LineType>
class LinesSet;
class OpenPolyline;

/*!
 * \brief Base class for all geometry containers representing a set of polylines.
 */
template<class LineType>
class LinesSet
{
private:
    std::vector<LineType> lines_;

public:
    LinesSet() = default;

    LinesSet(const LinesSet& other) = default;

    LinesSet(LinesSet&& other) = default;

    LinesSet(const std::vector<LineType>& lines)
        : lines_(lines)
    {
    }

    LinesSet(std::vector<LineType>&& lines)
        : lines_(std::move(lines))
    {
    }

    LinesSet(ClipperLib::Paths&& paths);

    const std::vector<LineType>& getLines() const
    {
        return lines_;
    }

    std::vector<LineType>& getLines()
    {
        return lines_;
    }

    void setLines(std::vector<LineType>&& lines)
    {
        lines_ = lines;
    }

    std::vector<LineType>::const_iterator begin() const
    {
        return lines_.begin();
    }

    std::vector<LineType>::iterator begin()
    {
        return lines_.begin();
    }

    std::vector<LineType>::const_iterator end() const
    {
        return lines_.end();
    }

    std::vector<LineType>::iterator end()
    {
        return lines_.end();
    }

    const LineType& back() const
    {
        return lines_.back();
    }

    LineType& back()
    {
        return lines_.back();
    }

    const LineType& front() const
    {
        return lines_.front();
    }

    LineType& front()
    {
        return lines_.front();
    }

    void push_back(const LineType& line, bool checkNonEmpty = false);

    void push_back(LineType&& line, bool checkNonEmpty = false);

    void push_back(ClipperLib::Paths&& paths);

    template<class OtherLineType>
    void push_back(LinesSet<OtherLineType>&& lines_set);

    void push_back(const LinesSet& other)
    {
        lines_.insert(lines_.end(), other.lines_.begin(), other.lines_.end());
    }

    void pop_back()
    {
        lines_.pop_back();
    }

    size_t size() const
    {
        return lines_.size();
    }

    bool empty() const
    {
        return lines_.empty();
    }

    void reserve(size_t size)
    {
        lines_.reserve(size);
    }

    void resize(size_t size)
    {
        lines_.resize(size);
    }

    void clear()
    {
        lines_.clear();
    }

    template<typename... Args>
    void emplace_back(Args&&... args)
    {
        lines_.emplace_back(args...);
    }

    std::vector<LineType>::iterator erase(std::vector<LineType>::const_iterator first, std::vector<LineType>::const_iterator last)
    {
        return lines_.erase(first, last);
    }

    LinesSet& operator=(LinesSet&& other)
    {
        lines_ = std::move(other.lines_);
        return *this;
    }

    LinesSet& operator=(const LinesSet& other)
    {
        lines_ = other.lines_;
        return *this;
    }

    LineType& operator[](size_t index)
    {
        return lines_[index];
    }

    const LineType& operator[](size_t index) const
    {
        return lines_[index];
    }

    LineType& newLine()
    {
        lines_.emplace_back();
        return lines_.back();
    }

    //!< Return the amount of points in all lines
    size_t pointCount() const;

    /*!
     * Remove a line from the list and move the last line to its place
     *
     * \warning changes the order of the lines!
     */
    void removeAt(size_t index);

    /*!
     * Add a simple line consisting of two points
     */
    void addSegment(const Point2LL& from, const Point2LL& to);

    coord_t length() const;

    void splitIntoSegments(LinesSet<OpenPolyline>& result) const;
    LinesSet<OpenPolyline> splitIntoSegments() const;

    /*!
     * Removes overlapping consecutive line segments which don't delimit a
     * positive area.
     *
     * This function is meant to work on polygons, not polylines. When misused
     * on polylines, it may cause too many vertices to be removed.
     * See \ref removeDegenerateVertsPolyline for a version that works on
     * polylines.
     */
    void removeDegenerateVerts();

    Shape offset(coord_t distance, ClipperLib::JoinType join_type = ClipperLib::jtMiter, double miter_limit = 1.2) const;

    /*!
     * Utility method for creating the tube (or 'donut') of a shape.
     * \param inner_offset Offset relative to the original shape-outline towards the inside of the shape. Sort-of like a negative normal offset, except it's the offset part that's
     * kept, not the shape. \param outer_offset Offset relative to the original shape-outline towards the outside of the shape. Comparable to normal offset. \return The resulting
     * polygons.
     */
    Shape tubeShape(const coord_t inner_offset, const coord_t outer_offset) const;

    void addPaths(ClipperLib::Clipper& clipper, ClipperLib::PolyType PolyTyp) const;

    void addPaths(ClipperLib::ClipperOffset& clipper, ClipperLib::JoinType jointType, ClipperLib::EndType endType) const;
};

} // namespace cura

#endif // GEOMETRY_LINES_SET_H
