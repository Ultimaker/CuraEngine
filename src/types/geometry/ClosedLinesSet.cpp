// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/geometry/ClosedLinesSet.h"

#include "types/geometry/MixedLinesSet.h"
#include "types/geometry/OpenPolyline.h"
#include "types/geometry/Shape.h"

namespace curaengine::types::geometry
{

MixedLinesSet ClosedLinesSet::intersection(const Shape& shape) const
{
    if (empty() || shape.empty())
    {
        return {};
    }

    ClipperLib::PolyTree result;
    ClipperLib::Clipper clipper(clipper_init);
    shape.addPaths(clipper, ClipperLib::ptSubject);
    addPaths(clipper, ClipperLib::ptClip);
    clipper.Execute(ClipperLib::ctIntersection, result);
    ClipperLib::Paths result_paths;
    ClipperLib::OpenPathsFromPolyTree(result, result_paths);

    return MixedLinesSet(std::move(result));
}

[[nodiscard]] MixedLinesSet ClosedLinesSet::difference(const Shape& shape) const
{
    if (empty())
    {
        return {};
    }
    if (shape.empty())
    {
        return MixedLinesSet(*this);
    }

    ClipperLib::PolyTree ret;
    ClipperLib::Clipper clipper(clipper_init);
    addPaths(clipper, ClipperLib::ptSubject);
    shape.addPaths(clipper, ClipperLib::ptClip);
    clipper.Execute(ClipperLib::ctDifference, ret);

    return MixedLinesSet(std::move(ret));
}

} // namespace curaengine::types::geometry
