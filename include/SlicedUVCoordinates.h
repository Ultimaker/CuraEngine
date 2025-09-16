// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SLICEDUVCOORDINATES_H
#define SLICEDUVCOORDINATES_H

#include <optional>
#include <unordered_map>
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

    /*! /!\ WARNINGS /!\
     *  - Currently assumes straight spans in UV-space will be over at most 2 faces, but this isn't _universally_ true.
     *  - Currently returns the UV-coords of the _entire_ segment from and to are part of,
     *    even if from and to repressent some points other than begin and end (though it should handle reversed).
     */
    std::optional<std::pair<Point2F, Point2F>> getUVCoordsLineSegment(const Point2LL& from, const Point2LL& to) const;

private:
    struct Segment
    {
        Point2LL start, end;
        Point2F uv_start, uv_end;
    };
    struct SegmentLocator
    {
    public:
        std::pair<Point2LL, Point2LL> operator()(Segment* const& seg)
        {
            return { seg->start, seg->end };
        }
    };

    static constexpr coord_t cell_size{ 1000 };
    static constexpr coord_t search_radius{ 1000 };

    std::vector<Segment> segments_;
    SparseLineGrid<Segment*, SegmentLocator> located_uv_coords_segs_;
    std::unordered_multimap<Point2LL, Segment*> segs_by_point_;
};

} // namespace cura
#endif // SLICEDUVCOORDINATES_H
