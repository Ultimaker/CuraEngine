// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <optional>

#include "types/geometry/Point2LL.h"

namespace curaengine::types::geometry
{
class PointMatrix;
}

namespace curaengine::types::geometry
{

enum class SegmentOverlappingType;
struct SegmentOverlapping;


/*! Base structure tha represents a segment that has been transformed in the space where the bridging lines are horizontal */
class TransformedSegment
{
public:
    TransformedSegment() = default;

    TransformedSegment(const types::geometry::Point2LL& transformed_start, const types::geometry::Point2LL& transformed_end)
        : start_(transformed_start)
        , end_(transformed_end)
    {
        updateMinMax();
    }

    TransformedSegment(const types::geometry::Point2LL& start, const types::geometry::Point2LL& end, const types::geometry::PointMatrix& matrix);

    void setStart(const types::geometry::Point2LL& transformed_start)
    {
        start_ = transformed_start;
        updateMinMax();
        length_.reset();
    }

    void setEnd(const types::geometry::Point2LL& transformed_end)
    {
        end_ = transformed_end;
        updateMinMax();
        length_.reset();
    }

    void updateMinMax();

    types::coord_t minY() const
    {
        return min_y_;
    }

    types::coord_t maxY() const
    {
        return max_y_;
    }

    const types::geometry::Point2LL& getStart() const
    {
        return start_;
    }

    const types::geometry::Point2LL& getEnd() const
    {
        return end_;
    }

    const types::coord_t length() const
    {
        if (! length_.has_value())
        {
            const_cast<TransformedSegment*>(this)->length_ = types::geometry::vSize(end_ - start_);
        }

        return length_.value();
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
    void cropTop(const types::coord_t new_max_y);

    /*! Move the segment minimum Y coordinate up, actually cropping it */
    void cropBottom(const types::coord_t new_min_y);

    /*! Calculate an overlapping type, given whether the segments overlap on the top, bottom or both. Only in case there is no segment intersection. */
    static SegmentOverlappingType makeNonIntersectingOverlapping(const bool overlap_top, const bool overlap_bottom);

private:
    types::geometry::Point2LL start_;
    types::geometry::Point2LL end_;
    types::coord_t min_y_{ 0 };
    types::coord_t max_y_{ 0 };
    std::optional<types::coord_t> length_;
};

} // namespace curaengine::types::geometry
