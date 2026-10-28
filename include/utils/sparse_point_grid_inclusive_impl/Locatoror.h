// Copyright (c) 2026 Scott Lenser
// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "utils/sparse_point_grid_inclusive_impl/SparsePointGridInclusiveElem.h"

namespace curaengine::utils::sparse_point_grid_inclusive_impl
{

template<class T>
struct Locatoror
{
    geometry::Point2LL operator()(const SparsePointGridInclusiveElem<T>& elem)
    {
        return elem.point;
    }
};

} // namespace curaengine::utils::sparse_point_grid_inclusive_impl
