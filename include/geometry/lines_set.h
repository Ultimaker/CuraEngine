// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_LINES_SET_H
#define GEOMETRY_LINES_SET_H

#include <vector>

#include "geometry/point2ll.h"

namespace cura
{

class Shape;
class OpenPolyline;
template<class T>
class LinesSet;

/*!
 * \brief Base class for all geometry containers representing a set of polylines. All the polylines
 *        have to be of the same type, e.g. open polylines. Due to the nature of the Polyline
 *        classes, it is possible to cast a LinesSet directly to a
 *        std::vector<std::vector<ClipperLib::IntPoint>> so that we can call the Clipper functions
 *        which having to do an active data conversion. It is also possible to virtually change the
 *        inner type of lines without having to do a conversion.
 */
template<class LineType>
class LinesSet : public std::vector<LineType>
{
public:
    LinesSet() = default;

    LinesSet(const LinesSet& other) = default;

    LinesSet(LinesSet&& other) = default;

    LinesSet(const std::vector<LineType>& lines)
        : std::vector<LineType>(lines)
    {
    }

    LinesSet(std::vector<LineType>&& lines)
        : std::vector<LineType>(std::move(lines))
    {
    }

    LinesSet(const std::vector<std::vector<Point2LL>>& paths)
        : std::vector<LineType>(*reinterpret_cast<const std::vector<LineType>*>(&paths))
    {
    }

    LinesSet& operator=(const LinesSet& other)
    {
        std::vector<LineType>::operator=(other);
        return *this;
    }

    LinesSet& operator=(LinesSet&& other)
    {
        std::vector<LineType>::operator=(other);
        return *this;
    }

    template<class OtherLineType>
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
    }

    void add(const LinesSet& other)
    {
        this->insert(this->end(), other.begin(), other.end());
    }

    void addIfNotEmpty(const LineType& line);

    void addIfNotEmpty(LineType&& line);

    LineType& newLine()
    {
        this->emplace_back();
        return this->back();
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
    void addLine(const Point2LL& from, const Point2LL& to);

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

    Shape offset(coord_t distance, ClipperLib::JoinType joinType = ClipperLib::jtMiter, double miter_limit = 1.2) const;

    /*!
     * Utility method for creating the tube (or 'donut') of a shape.
     * \param inner_offset Offset relative to the original shape-outline towards the inside of the shape. Sort-of like a negative normal offset, except it's the offset part that's
     * kept, not the shape. \param outer_offset Offset relative to the original shape-outline towards the outside of the shape. Comparable to normal offset. \return The resulting
     * polygons.
     */
    Shape tubeShape(const coord_t inner_offset, const coord_t outer_offset) const;
};

} // namespace cura

#endif // GEOMETRY_LINES_SET_H
