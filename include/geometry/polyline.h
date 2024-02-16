// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_POLYLINE_H
#define GEOMETRY_POLYLINE_H

#include "geometry/points_set.h"
#include "geometry/polyline_type.h"
#include "geometry/segment_iterator.h"

namespace cura
{

class OpenPolyline;
template<class T>
class LinesSet;
class AngleRadians;

template<PolylineType PolylineTypeVal>
class Polyline : public PointsSet
{
    friend class Polygons;

public:
    static constexpr PolylineType type_ = PolylineTypeVal;

public:
    using segments_iterator = SegmentIterator<false>;
    using const_segments_iterator = SegmentIterator<true>;

    Polyline() = default;

    Polyline(const std::initializer_list<Point2LL>& initializer)
        : PointsSet(initializer)
    {
    }

    Polyline(const std::vector<Point2LL>& points)
        : PointsSet(points)
    {
    }

    Polyline(std::vector<Point2LL>&& points)
        : PointsSet(points)
    {
    }

    template<class OtherType>
    OtherType& toType()
    {
        return *reinterpret_cast<OtherType*>(this);
    }

    template<class OtherType>
    const OtherType& toType() const
    {
        return *reinterpret_cast<const OtherType*>(this);
    }

    /*Polyline& operator=(const Polyline& other)
    {
        std::vector<point_t>::operator=(other);
        return *this;
    }*/

    const_segments_iterator beginSegments() const;

    const_segments_iterator endSegments() const;

    segments_iterator beginSegments();

    segments_iterator endSegments();

    /*!
     * Split these poly line objects into several line segment objects consisting of only two verts
     * and store them in the \p result
     */
    void splitIntoSegments(LinesSet<OpenPolyline>& result) const;
    LinesSet<OpenPolyline> splitIntoSegments() const;

    /*!
     * On Y-axis positive upward displays, Orientation will return true if the polygon's orientation is counter-clockwise.
     *
     * from http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/Orientation.htm
     */
    bool orientation() const
    {
        return ClipperLib::Orientation(*this);
    }

    coord_t length() const;

    bool shorterThan(const coord_t check_length) const;

    void reverse()
    {
        ClipperLib::ReversePath(*this);
    }

    void removeColinearEdges(const AngleRadians max_deviation_angle);

    /*!
     * Removes consecutive line segments with same orientation and changes this polygon.
     *
     * 1. Removes verts which are connected to line segments which are too small.
     * 2. Removes verts which detour from a direct line from the previous and next vert by a too small amount.
     * 3. Moves a vert when a small line segment is connected to a much longer one. in order to maintain the outline of the object.
     * 4. Don't remove a vert when the impact on the outline of the object is too great.
     *
     * Note that the simplify is a best effort algorithm. It does not guarantee that no lines below the provided smallest_line_segment_squared are left.
     *
     * The following example (Two very long line segments (" & , respectively) that are connected by a very small line segment (i) is unsimplifable by this
     * function, even though the actual area change of removing line segment i is very small. The reason for this is that in the case of long lines, even a small
     * deviation from it's original direction is very noticeable in the final result, especially if the polygons above make a slightly different choice.
     *
     * """"""""""""""""""""""""""""""""i,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

     *
     * \param smallest_line_segment_squared maximal squared length of removed line segments
     * \param allowed_error_distance_squared The square of the distance of the middle point to the line segment of the consecutive and previous point for which the middle point is
     removed
     */
    void simplify(const coord_t smallest_line_segment_squared = MM2INT(0.01) * MM2INT(0.01), const coord_t allowed_error_distance_squared = 25);

    /*!
     * See simplify(.)
     */
#warning This can probably be merge with simplify ?
    void simplifyPolyline(const coord_t smallest_line_segment_squared = 100, const coord_t allowed_error_distance_squared = 25);

private:
    /*!
     * Private implementation for both simplify and simplifyPolygons.
     *
     * Made private to avoid accidental use of the wrong function.
     */
    void _simplify(const coord_t smallest_line_segment_squared = 100, const coord_t allowed_error_distance_squared = 25, bool processing_polylines = false);
};

} // namespace cura

#endif // GEOMETRY_POLYLINE_H
