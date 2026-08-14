// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef BRIDGE_TRANSFORMEDSEGMENT_H
#define BRIDGE_TRANSFORMEDSEGMENT_H

#include <optional>

#include "geometry/Point2LL.h"

namespace cura
{

enum class SegmentOverlappingType;
struct SegmentOverlapping;
class PointMatrix;

/*! Base structure tha represents a segment that has been transformed in the space where the bridging lines are horizontal */
class TransformedSegment
{
public:
    TransformedSegment() = default;

    TransformedSegment(const Point2LL& transformed_start, const Point2LL& transformed_end)
        : start_(transformed_start)
        , end_(transformed_end)
    {
        updateMinMax();
    }

    TransformedSegment(const Point2LL& start, const Point2LL& end, const PointMatrix& matrix);

    void setStart(const Point2LL& transformed_start)
    {
        start_ = transformed_start;
        updateMinMax();
    }

    void setEnd(const Point2LL& transformed_end)
    {
        end_ = transformed_end;
        updateMinMax();
    }

    void updateMinMax();

    coord_t minY() const
    {
        return min_y_;
    }

    coord_t maxY() const
    {
        return max_y_;
    }

    const Point2LL& getStart() const
    {
        return start_;
    }

    const Point2LL& getEnd() const
    {
        return end_;
    }

    /*!
     * Calculate the horizontal overlapping between this segment and an other segment, given the expand direction. Overlapping means that the new segment is on the right (or left)
     * of the actual segment on a horizontal band formed by the segment top and bottom
     * @param other The other segment that may be overlapping
     * @param expand_direction The expand direction, 1 means expand to the right, -1 expand to the left
     * @return The overlapping details if the segment actually overlap, or nullopt if it doesn't
     */
    std::optional<SegmentOverlapping> calculateOverlapping(const TransformedSegment& other, const int8_t expand_direction) const;

    /*! Move the segment maximum Y coordinate down, actually cropping it */
    void cropTop(const coord_t new_max_y);

    /*! Move the segment minimum Y coordinate up, actually cropping it */
    void cropBottom(const coord_t new_min_y);

    /*! Calculate an overlapping type, given whether the segments overlap on the top, bottom or both. Only in case there is no segment intersection. */
    static SegmentOverlappingType makeNonIntersectingOverlapping(const bool overlap_top, const bool overlap_bottom);

private:
    Point2LL start_;
    Point2LL end_;
    coord_t min_y_{ 0 };
    coord_t max_y_{ 0 };
};

} // namespace cura

#endif
