// Copyright (c) 2026 UltiMaker
// Copyright (c) 2026 Scott Lenser
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "types/geometry/sparse_point_grid_inclusive_impl/SparsePointGridInclusiveElem.h"

namespace curaengine::types::geometry::sparse_point_grid_inclusive_impl
{

template<class T>
struct Locatoror
{
    types::geometry::Point2LL operator()(const SparsePointGridInclusiveElem<T>& elem)
    {
        return elem.point;
    }
};

} // namespace curaengine::types::geometry::sparse_point_grid_inclusive_impl
