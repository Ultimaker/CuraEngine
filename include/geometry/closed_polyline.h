// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_CLOSED_POLYLINE_H
#define GEOMETRY_CLOSED_POLYLINE_H

#include "geometry/polyline.h"

namespace cura
{

class OpenPolyline;

class ClosedPolyline : public Polyline
{
private:
    /*! If true, that means the last point in the list is at the same position as the start point,
     *  making it explicitely closed.
     *  If false, you have to add an additional segment between the end point and the start point to
     *  actually have the line closed. */
    bool explicitely_closed_{ false };

public:
    ClosedPolyline() = default;

    ClosedPolyline(const ClosedPolyline& other) = default;

    ClosedPolyline(ClosedPolyline&& other) = default;

    ClosedPolyline(const std::initializer_list<Point2LL>& initializer, bool explicitely_closed)
        : Polyline(initializer)
        , explicitely_closed_(explicitely_closed)
    {
    }

    explicit ClosedPolyline(const ClipperLib::Path& points, bool explicitely_closed)
        : Polyline(points)
        , explicitely_closed_(explicitely_closed)
    {
    }

    explicit ClosedPolyline(ClipperLib::Path&& points, bool explicitely_closed)
        : Polyline(points)
        , explicitely_closed_(explicitely_closed)
    {
    }

    virtual bool addClosingSegment() const
    {
        return ! explicitely_closed_;
    }

    virtual size_t segmentsCount() const override
    {
        if (explicitely_closed_)
        {
            return size() >= 3 ? size() - 1 : 0;
        }
        else
        {
            return size() >= 2 ? size() : 0;
        }
    }

    ClosedPolyline& operator=(const ClosedPolyline& other)
    {
        Polyline::operator=(other);
        return *this;
    }

    ClosedPolyline& operator=(ClosedPolyline&& other)
    {
        Polyline::operator=(other);
        return *this;
    }

    bool isExplicitelyClosed() const
    {
        return explicitely_closed_;
    }

    /*!
     * \brief Sets whether the points set is to be treated as explicitely or implicitely closed
     * \warning This does not actually changes the points set, only the interpretation of it will
     *          change. So use this method only if you really know what you are doing.
     */
    void setExplicitelyClosed(bool explicitely_closed)
    {
        explicitely_closed_ = explicitely_closed;
    }

    /*!
     * Check if we are inside the polygon. We do this by tracing from the point towards the positive X direction,
     * every line we cross increments the crossings counter. If we have an even number of crossings then we are not inside the polygon.
     * Care needs to be taken, if p.Y exactly matches a vertex to the right of p, then we need to count 1 intersect if the
     * outline passes vertically past; and 0 (or 2) intersections if that point on the outline is a 'top' or 'bottom' vertex.
     * The easiest way to do this is to break out two cases for increasing and decreasing Y ( from p0 to p1 ).
     * A segment is tested if pa.Y <= p.Y < pb.Y, where pa and pb are the points (from p0,p1) with smallest & largest Y.
     * When both have the same Y, no intersections are counted but there is a special test to see if the point falls
     * exactly on the line.
     *
     * Returns false if outside, true if inside; if the point lies exactly on the border, will return 'border_result'.
     *
     * \deprecated This function is no longer used, since the Clipper function is used by the function PolygonRef::inside(.)
     *
     * \param p The point for which to check if it is inside this polygon
     * \param border_result What to return when the point is exactly on the border
     * \return Whether the point \p p is inside this polygon (or \p border_result when it is on the border)
     */
    // bool _inside(Point2LL p, bool border_result = false) const;

    /*!
     * Clipper function.
     * Returns false if outside, true if inside; if the point lies exactly on the border, will return 'border_result'.
     *
     * http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/PointInPolygon.htm
     */
    bool inside(const Point2LL& p, bool border_result = false) const;

    bool inside(const ClipperLib::Path& polygon) const;

    /*!
     * \brief Converts the closed polyline to an open polyline which happens to have its end and start points at the same
     *        position, making it a pseudo-closed polyline. Although this should never be required in practice, there
     *        are many places in the code where this is done because historically we wouldn't make a clear difference
     *        between open and closed polylines
     * \return An open polyline instance, with the end point at the same position of the start point
     */
    OpenPolyline toPseudoOpenPolyline() const;
};

} // namespace cura

#endif // GEOMETRY_CLOSED_POLYLINE_H
