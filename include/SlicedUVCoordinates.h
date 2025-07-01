// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SLICEDUVCOORDINATES_H
#define SLICEDUVCOORDINATES_H

#include <optional>

#include "geometry/Point2LL.h"
#include "utils/Point2F.h"
#include "utils/SparsePointGridInclusive.h"

namespace cura
{

class Image;
class SlicerSegment;

class SlicedUVCoordinates
{
public:
    explicit SlicedUVCoordinates(const std::vector<SlicerSegment>& segments);

    std::optional<Point2F> getUVCoordinatesOfClosedPoint(const Point2LL& position) const;

private:
    static constexpr coord_t cell_size{ 1000 };
    static constexpr coord_t search_radius{ 1000 };
    SparsePointGridInclusive<Point2F> located_uv_coordinates_;
};

} // namespace cura
#endif // SLICEDUVCOORDINATES_H
