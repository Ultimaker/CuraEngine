// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SLICEDUVCOORDINATES_H
#define SLICEDUVCOORDINATES_H

#include <optional>
#include <utility>

#include "geometry/Point2LL.h"
#include "utils/Point2F.h"
#include "utils/SparseLineGrid.h"

namespace cura
{

class Image;
class SlicerSegment;

class SlicedUVCoordinates
{
public:
    explicit SlicedUVCoordinates(const std::vector<SlicerSegment>& segments);

    std::optional<Point2F> getClosestUVCoordinates(const Point2LL& position) const;

private:
    struct Segment
    {
        Point2LL start, end;
        Point2F uv_start, uv_end;
    };
    struct SegmentLocator
    {
    public:
        std::pair<Point2LL, Point2LL> operator() (const Segment& seg)
        {
            return {seg.start, seg.end};
        }
    };

    static constexpr coord_t cell_size{ 1000 };
    static constexpr coord_t search_radius{ 1000 };

    SparseLineGrid<Segment, SegmentLocator> located_uv_coords_segs_;
};

} // namespace cura
#endif // SLICEDUVCOORDINATES_H
