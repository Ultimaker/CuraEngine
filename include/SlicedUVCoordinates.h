// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <optional>

#include "geometry/Point2F.h"
#include "geometry/Point2LL.h"
#include "utils/SparsePointGridInclusive.h"

namespace curaengine
{

class Image;
class SlicerSegment;

class SlicedUVCoordinates
{
public:
    explicit SlicedUVCoordinates(const std::vector<SlicerSegment>& segments);

    std::optional<::curaengine::geometry::Point2F> getClosestUVCoordinates(const ::curaengine::geometry::Point2LL& position) const;

private:
    struct Segment
    {
        ::curaengine::geometry::Point2LL start, end;
        ::curaengine::geometry::Point2F uv_start, uv_end;
    };

    static constexpr ::curaengine::utils::coord_t cell_size{ 1000 };
    static constexpr ::curaengine::utils::coord_t search_radius{ 1000 };
    ::curaengine::utils::SparsePointGridInclusive<::curaengine::geometry::Point2F> located_uv_coordinates_;
    std::vector<Segment> segments_;
};

} // namespace curaengine
