// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PathAdapter.h"

#include "utils/ExtrusionLine.h"

namespace curaengine
{

template<>
const ::curaengine::geometry::Point2LL& PathAdapter<::curaengine::utils::ExtrusionLine>::pointAt(size_t index) const
{
    return path_.junctions_.at(index).p_;
}

template<>
::curaengine::utils::coord_t PathAdapter<::curaengine::utils::ExtrusionLine>::lineWidthAt(size_t index) const
{
    return path_.junctions_.at(index).w_;
}

template<>
const ::curaengine::geometry::Point2LL& PathAdapter<::curaengine::geometry::Polygon>::pointAt(size_t index) const
{
    return path_.at(index);
}

template<>
::curaengine::utils::coord_t PathAdapter<::curaengine::geometry::Polygon>::lineWidthAt(size_t /*index*/) const
{
    return fixed_line_width_;
}

} // namespace curaengine
