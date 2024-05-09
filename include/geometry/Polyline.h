// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_POLYLINE_H
#define GEOMETRY_POLYLINE_H

#include "geometry/OpenLinesSet.h"
#include "geometry/PointsSet.h"
#include "geometry/SegmentIterator.h"

namespace cura
{

template<class LineType>
class LinesSet;
class AngleRadians;
class OpenPolyline;

/*!
 * \brief Base class for various types of polylines. A polyline is basically a set of points, but
 *        we geometrically interpret them forming a chain of segments between each other.
 *  \sa https://github.com/Ultimaker/CuraEngine/wiki/Geometric-Base-Types#pointsset
 *
 *  * Open Polyline : this represents a line that does not close, i.e. the last point is different
 *                    from the initial point (think of the U letter)
 *  * Closed Polyline : a closed polyline has a final segment joining the last point and the
 *                      initial one (think of the O letter)
 *  * Polygon : this is a particular type of closed polyline, for which we consider that the
 *              "inside" part of the line forms a surface
 *
 *  \note Historically, the open and closed polylines were not explicitely differenciated, so
 *        sometimes we would use an open polyline with an extra point at the end, which virtually
 *        closes the line. This behaviour is now deprecated and should be removed over time.
 */
class Polyline : public PointsSet
{
public:
    using segments_iterator = SegmentIterator<ConstnessType::Modifiable>;
    using const_segments_iterator = SegmentIterator<ConstnessType::Const>;

    /*! \brief Builds an empty polyline */
    Polyline() = default;

    /*! \brief Creates a copy of the given polyline */
    Polyline(const Polyline& other) = default;

    /*! \brief Constructor that takes ownership of the inner points list from the given polyline */
    Polyline(Polyline&& other) = default;

    /*! \brief Constructor with a points initializer list, provided for convenience */
    Polyline(const std::initializer_list<Point2LL>& initializer)
        : PointsSet(initializer)
    {
    }

    /*! \brief Constructor with an existing list of points */
    explicit Polyline(const ClipperLib::Path& points)
        : PointsSet(points)
    {
    }

    /*! \brief Constructor that takes ownership of the given list of points */
    explicit Polyline(ClipperLib::Path&& points)
        : PointsSet{ std::move(points) }
    {
    }

    ~Polyline() override = default;

    /*!
     * \brief Indicates whether this polyline has an additional closing segment between the last
     *        point in the set and the first one
     * \return  True if a segment between the last and first point should be considered
     */
    [[nodiscard]] virtual bool hasClosingSegment() const = 0;

    /*!
     * \brief Gets the total number of "full" segments in the polyline. Calling this is also safe if
     *        there are not enough points to make a valid polyline, so it can also be a good
     *        indicator of a "valid" polyline.
     */
    [[nodiscard]] virtual size_t segmentsCount() const = 0;

    /*!
     * \brief Indicates whether the points set form a valid polyline, i.e. if it has enough points
     *        according to its type.
     */
    [[nodiscard]] virtual bool isValid() const = 0;

    Polyline& operator=(const Polyline& other) = default;

    Polyline& operator=(Polyline&& other) = default;

    /*! \brief Provides a begin iterator to iterate over all the segments of the line */
    [[nodiscard]] const_segments_iterator beginSegments() const;

    /*! \brief Provides an end iterator to iterate over all the segments of the line */
    [[nodiscard]] const_segments_iterator endSegments() const;

    /*! \brief Provides a begin iterator to iterate over all the segments of the line */
    segments_iterator beginSegments();

    /*! \brief Provides an end iterator to iterate over all the segments of the line */
    segments_iterator endSegments();

    /*!
     * Split these poly line objects into several line segment objects consisting of only two verts
     * and store them in the \p result
     */
    void splitIntoSegments(OpenLinesSet& result) const;
    [[nodiscard]] OpenLinesSet splitIntoSegments() const;

    /*!
     * On Y-axis positive upward displays, Orientation will return true if the polygon's orientation is counter-clockwise.
     *
     * from http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/Orientation.htm
     */
    [[nodiscard]] bool orientation() const
    {
        return ClipperLib::Orientation(getPoints());
    }

    [[nodiscard]] coord_t length() const;

    [[nodiscard]] bool shorterThan(const coord_t check_length) const;

    void reverse()
    {
        ClipperLib::ReversePath(getPoints());
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
};

} // namespace cura

#endif // GEOMETRY_POLYLINE_H
