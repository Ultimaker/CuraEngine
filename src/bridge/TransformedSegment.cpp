// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "bridge/TransformedSegment.h"

#include <algorithm>

#include "bridge/SegmentOverlapping.h"
#include "bridge/SegmentOverlappingData.h"
#include "bridge/SegmentOverlappingType.h"
#include "geometry/PointMatrix.h"
#include "utils/linearAlg2D.h"
#include "utils/math.h"


namespace cura
{

TransformedSegment::TransformedSegment(const Point2LL& start, const Point2LL& end, const PointMatrix& matrix)
    : TransformedSegment(matrix.apply(start), matrix.apply(end))
{
}

void TransformedSegment::updateMinMax()
{
    std::tie(min_y_, max_y_) = std::minmax(start_.Y, end_.Y);
}

void TransformedSegment::cropTop(const coord_t new_max_y_)
{
    setEnd(Point2LL(LinearAlg2D::lineHorizontalLineIntersection(start_, end_, new_max_y_).value_or(0), new_max_y_));
}

/*! Move the segment minimum Y coordinate up, actually cropping it */
void TransformedSegment::cropBottom(const coord_t new_min_y_)
{
    setStart(Point2LL(LinearAlg2D::lineHorizontalLineIntersection(start_, end_, new_min_y_).value_or(0), new_min_y_));
}

std::optional<SegmentOverlapping> TransformedSegment::calculateOverlapping(const TransformedSegment& other, const int8_t expand_direction) const
{
    if (fuzzy_is_greater_or_equal(other.min_y_, max_y_) || fuzzy_is_lesser_or_equal(other.max_y_, min_y_))
    {
        // Not on the same horizontal band, or very slightly overlapping , discard
        return std::nullopt;
    }

    const SegmentOverlappingData overlapping(min_y_, max_y_, other.min_y_, other.max_y_, &start_, &end_, other.start_, other.end_);

    if (fuzzy_equal(overlapping.this_x_min, overlapping.other_x_min) && fuzzy_equal(overlapping.this_x_max, overlapping.other_x_max))
    {
        // Segments are on top of each other, discard
        return std::nullopt;
    }

    int8_t sign_min = sign(overlapping.other_x_min - overlapping.this_x_min);
    int8_t sign_max = sign(overlapping.other_x_max - overlapping.this_x_max);
    const bool overlap_top = (overlapping.y_max == max_y_);
    const bool overlap_bottom = (overlapping.y_min == min_y_);

    TransformedSegment line_part = overlapping.makeOtherOverlappingPart();

    if (sign_min != sign_max)
    {
        // Segments are apparently intersecting each other
        float intersection_segment;
        float intersection_infill_line;
        if (LinearAlg2D::segmentSegmentIntersection(start_, end_, other.start_, other.end_, intersection_segment, intersection_infill_line))
        {
            const Point2LL intersection = lerp(start_, end_, intersection_segment);
            if (intersection.Y >= overlapping.y_max - EPSILON)
            {
                // Intersection is very close from top, consider as if not intersecting
                sign_max = sign_min;
            }
            else if (intersection.Y <= overlapping.y_min + EPSILON)
            {
                // Intersection is very close from bottom, consider as if not intersecting
                sign_min = sign_max;
            }
            else
            {
                SegmentOverlappingType type;

                if (sign_max == expand_direction)
                {
                    type = overlap_top ? SegmentOverlappingType::Top : SegmentOverlappingType::Middle;
                    line_part.setStart(intersection);
                }
                else
                {
                    type = overlap_bottom ? SegmentOverlappingType::Bottom : SegmentOverlappingType::Middle;
                    line_part.setEnd(intersection);
                }

                return SegmentOverlapping{ type, line_part };
            }
        }
    }

    if (sign_min != expand_direction)
    {
        // Segment is on the non-expanding side, discard
        return std::nullopt;
    }
    else
    {
        return SegmentOverlapping{ makeNonIntersectingOverlapping(overlap_top, overlap_bottom), line_part };
    }
}

SegmentOverlappingType TransformedSegment::makeNonIntersectingOverlapping(const bool overlap_top, const bool overlap_bottom)
{
    if (overlap_top && overlap_bottom)
    {
        return SegmentOverlappingType::Full;
    }
    else if (overlap_bottom)
    {
        return SegmentOverlappingType::Bottom;
    }
    else if (overlap_top)
    {
        return SegmentOverlappingType::Top;
    }
    else
    {
        return SegmentOverlappingType::Middle;
    }
}

} // namespace cura
