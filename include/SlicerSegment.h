// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <optional>

#include "geometry/Point2F.h"
#include "geometry/Point2LL.h"

namespace cura
{
class MeshVertex;

class SlicerSegment
{
public:
    Point2LL start, end;
    std::optional<Point2F> uv_start, uv_end;
    int faceIndex = -1;
    // The index of the other face connected via the edge that created end
    int endOtherFaceIdx = -1;
    // If end corresponds to a vertex of the mesh, then this is populated
    // with the vertex that it ended on.
    const MeshVertex* endVertex = nullptr;
    bool addedToPolygon = false;
};

} // namespace cura
