// Copyright (c) 2026 Scott Lenser
// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "geometry/Point2LL.h"

namespace cura::SparsePointGridInclusiveImpl
{

template<class Val>
struct SparsePointGridInclusiveElem
{
    SparsePointGridInclusiveElem()
    {
    }

    SparsePointGridInclusiveElem(const Point2LL& point_, const Val& val_)
        : point(point_)
        , val(val_)
    {
    }

    Point2LL point;
    Val val;
};

} // namespace cura::SparsePointGridInclusiveImpl
