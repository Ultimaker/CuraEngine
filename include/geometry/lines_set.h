// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_LINES_SET_H
#define GEOMETRY_LINES_SET_H

#include <vector>

#include "geometry/point2ll.h"
#include "geometry/polyline_type.h"

namespace cura
{

class Shape;
class Polyline;

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

    /*LinesSet(const std::vector<std::vector<Point2LL>>& paths)
        : std::vector<LineType>(*reinterpret_cast<const std::vector<LineType>*>(&paths))
    {
    }*/

    LinesSet(PolylineType type, std::vector<std::vector<Point2LL>>&& paths);

    const std::vector<LineType>& getLines() const
    {
        return lines_;
    }

    std::vector<LineType>& getLines()
    {
        return lines_;
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

    void push_back(const LineType& line, bool checkNonEmpty = false);

    void push_back(LineType&& line, bool checkNonEmpty = false);

    void push_back(PolylineType type, ClipperLib::Paths&& paths);

    void push_back(LinesSet<LineType>&& lines_set);

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

    template<typename... Args>
    void emplace_back(Args&&... args)
    {
        lines_.emplace_back(args...);
    }

    LinesSet& operator=(const LinesSet& other) = default;

    LinesSet& operator=(LinesSet&& other) = default;

    LineType& operator[](size_t index)
    {
        return lines_[index];
    }

    const LineType& operator[](size_t index) const
    {
        return lines_[index];
    }

#warning rename this to asType
    /*template<class OtherLineType>
    const LinesSet<OtherLineType>& toType() const
    {
        // This does work as long as we don't add any attribute to the PointsSet class or any of its child
        return *reinterpret_cast<const LinesSet<OtherLineType>*>(this);
    }

    const std::vector<std::vector<Point2LL>>& asRawVector() const
    {
        // This does work as long as we don't add any attribute to the PointsSet class or any of its child
        return *reinterpret_cast<const std::vector<std::vector<Point2LL>>*>(this);
    }

    std::vector<std::vector<Point2LL>>& asRawVector()
    {
        // This does work as long as we don't add any attribute to the PointsSet class or any of its child
        return *reinterpret_cast<std::vector<std::vector<Point2LL>>*>(this);
    }*/

    void add(const LinesSet& other)
    {
        lines_->insert(other.lines_.end(), other.lines_.begin(), other.lines_.end());
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
#warning rename to addSegment and move to mixed
    void addLine(const Point2LL& from, const Point2LL& to);

    coord_t length() const;

    void splitIntoSegments(LinesSet<Polyline>& result) const;
    LinesSet<Polyline> splitIntoSegments() const;

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

    Shape offset(coord_t distance, ClipperLib::JoinType joinType = ClipperLib::jtMiter, double miter_limit = 1.2) const;

    /*!
     * Utility method for creating the tube (or 'donut') of a shape.
     * \param inner_offset Offset relative to the original shape-outline towards the inside of the shape. Sort-of like a negative normal offset, except it's the offset part that's
     * kept, not the shape. \param outer_offset Offset relative to the original shape-outline towards the outside of the shape. Comparable to normal offset. \return The resulting
     * polygons.
     */
    Shape tubeShape(const coord_t inner_offset, const coord_t outer_offset) const;

protected:
    void addPaths(ClipperLib::Clipper& clipper, ClipperLib::PolyType PolyTyp) const;
};

} // namespace cura

#endif // GEOMETRY_LINES_SET_H
