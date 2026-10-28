// Copyright (c) 2026 Scott Lenser
// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "geometry/Point2LL.h"

namespace curaengine::utils::sparse_point_grid_inclusive_impl
{

template<class Val>
struct SparsePointGridInclusiveElem
{
    SparsePointGridInclusiveElem()
    {
    }

    SparsePointGridInclusiveElem(const geometry::Point2LL& point_, const Val& val_)
        : point(point_)
        , val(val_)
    {
    }

    geometry::Point2LL point;
    Val val;
};

} // namespace curaengine::utils::sparse_point_grid_inclusive_impl
