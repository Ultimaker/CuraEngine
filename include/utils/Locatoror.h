// Copyright (c) 2026 Scott Lenser
// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "utils/SparsePointGridInclusiveElem.h"

namespace cura::SparsePointGridInclusiveImpl
{

template<class T>
struct Locatoror
{
    Point2LL operator()(const SparsePointGridInclusiveElem<T>& elem)
    {
        return elem.point;
    }
};

} // namespace cura::SparsePointGridInclusiveImpl
