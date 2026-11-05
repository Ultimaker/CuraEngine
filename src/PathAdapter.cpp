// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PathAdapter.h"

#include "types/geometry/ExtrusionLine.h"

namespace curaengine
{
using namespace types::geometry;
using namespace types;
using namespace utils;

template<>
const Point2LL& PathAdapter<ExtrusionLine>::pointAt(size_t index) const
{
    return path_.junctions_.at(index).p_;
}

template<>
coord_t PathAdapter<ExtrusionLine>::lineWidthAt(size_t index) const
{
    return path_.junctions_.at(index).w_;
}

template<>
const Point2LL& PathAdapter<Polygon>::pointAt(size_t index) const
{
    return path_.at(index);
}

template<>
coord_t PathAdapter<Polygon>::lineWidthAt(size_t /*index*/) const
{
    return fixed_line_width_;
}

} // namespace curaengine
