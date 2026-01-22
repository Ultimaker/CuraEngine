// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef BRIDGE_EXPANSIONRANGE_H
#define BRIDGE_EXPANSIONRANGE_H

#include <optional>

#include "bridge/TransformedSegment.h"

namespace cura
{

struct SegmentOverlapping;

/*! Helper structure containing a vertical range that has been projected (or not) for a segment */
class ExpansionRange
{
public:
    /*!
     * Constructs a range that has been projected to an infill line
     * @param segment The actual projected segment, cropped to the range
     * @param supporting_segment The infill line which is supporting this segment
     */
    ExpansionRange(const TransformedSegment& segment, const TransformedSegment* supporting_segment)
        : data_{ .segment = segment }
        , supporting_segment_(supporting_segment)
        , is_projected_(true)
    {
    }

    /*!
     * Constructs a range that has not been projected
     * @param min_y The range bottom Y coordinate
     * @param max_y The range maximum Y coordinate
     * @param expanded_segment The segment being expanded
     */
    ExpansionRange(const coord_t min_y, const coord_t max_y, const TransformedSegment* expanded_segment)
        : data_{ .range = { .min_y = min_y, .max_y = max_y } }
        , supporting_segment_(expanded_segment)
        , is_projected_(false)
    {
    }

    coord_t minY() const
    {
        return is_projected_ ? data_.segment.minY() : data_.range.min_y;
    }

    coord_t maxY() const
    {
        return is_projected_ ? data_.segment.maxY() : data_.range.max_y;
    }

    /*!
     * Calculate the horizontal overlapping between this range and an other segment, given the expand direction. Overlapping means that the new segment is on the right (or left)
     * of the actual segment on a horizontal band formed by the segment top and bottom
     * @param other The other segment that may be overlapping
     * @param expand_direction The expand direction, 1 means expand to the right, -1 expand to the left
     * @return The overlapping details if the segment actually overlap, or nullopt if it doesn't
     */
    std::optional<SegmentOverlapping> calculateOverlapping(const TransformedSegment& other, const int8_t expand_direction) const;

    /*! Move the range maximum Y coordinate down, actually cropping it */
    void cropTop(const coord_t new_max_y)
    {
        if (is_projected_)
        {
            data_.segment.cropTop(new_max_y);
        }
        else
        {
            data_.range.max_y = new_max_y;
        }
    }

    /*! Move the segment minimum Y coordinate up, actually cropping it */
    void cropBottom(const coord_t new_min_y)
    {
        if (is_projected_)
        {
            data_.segment.cropBottom(new_min_y);
        }
        else
        {
            data_.range.min_y = new_min_y;
        }
    }

    /*! Indicates if the range is still valid, e.g. doesn't have a (almost) null height */
    bool isValid() const
    {
        return fuzzy_not_equal(maxY(), minY());
    }

    void setProjectedEnd(const Point2LL& end)
    {
        data_.segment.setEnd(end);
    }

    const TransformedSegment* getSupportingSegment() const
    {
        return supporting_segment_;
    }

    bool isProjected() const
    {
        return is_projected_;
    }

    const TransformedSegment& getProjectedSegment() const
    {
        return data_.segment;
    }

private:
    union
    {
        // The segment that has been projected
        TransformedSegment segment;
        // The range that has not been projected
        struct
        {
            coord_t min_y;
            coord_t max_y;
        } range;
    } data_;

    // The supporting segment that this range is based upon, or the segment being expanded if it has not been projected
    const TransformedSegment* supporting_segment_;

    // True if this range has been projected to a supported infill line, in which case you can use data.segment. Otherwise the range has not been projected and only contains
    // its Y range in data.range.
    bool is_projected_;
};

} // namespace cura

#endif
