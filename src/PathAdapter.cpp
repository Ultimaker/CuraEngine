// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PathAdapter.h"

#include "utils/ExtrusionLine.h"

namespace cura
{

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

} // namespace cura
