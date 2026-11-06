// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <optional>

#include "types/geometry/Point2F.h"
#include "types/geometry/Point2LL.h"
#include "types/geometry/SparsePointGridInclusive.h"

namespace curaengine
{

class Image;
class SlicerSegment;

class SlicedUVCoordinates
{
public:
    explicit SlicedUVCoordinates(const std::vector<SlicerSegment>& segments);

    std::optional<types::geometry::Point2F> getClosestUVCoordinates(const types::geometry::Point2LL& position) const;

private:
    struct Segment
    {
        types::geometry::Point2LL start, end;
        types::geometry::Point2F uv_start, uv_end;
    };

    static constexpr types::coord_t cell_size{ 1000 };
    static constexpr types::coord_t search_radius{ 1000 };
    types::geometry::SparsePointGridInclusive<types::geometry::Point2F> located_uv_coordinates_;
    std::vector<Segment> segments_;
};

} // namespace curaengine
