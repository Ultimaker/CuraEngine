// Copyright (c) 2026 UltiMaker
// Copyright (c) 2026 Scott Lenser
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "types/geometry/Point2LL.h"

namespace curaengine::types::geometry::sparse_point_grid_inclusive_impl
{

template<class Val>
struct SparsePointGridInclusiveElem
{
    SparsePointGridInclusiveElem()
    {
    }

    SparsePointGridInclusiveElem(const types::geometry::Point2LL& point_, const Val& val_)
        : point(point_)
        , val(val_)
    {
    }

    types::geometry::Point2LL point;
    Val val;
};

} // namespace curaengine::types::geometry::sparse_point_grid_inclusive_impl
