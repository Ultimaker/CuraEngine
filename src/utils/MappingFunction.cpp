/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */

#include <cassert>

#include "MappingFunction.h"

namespace cura
{

std::vector<MappingFunction::real> MappingFunction::evaluateFunction(const std::function<real (real)>& func, real min, real max, uint_fast16_t n_points) const
{
    std::vector<real> ret(n_points, 0);
    for (uint_fast16_t idx = 0; idx < n_points; idx++)
    {
        ret[idx] = func(static_cast<real>(idx) / static_cast<real>(n_points) * (max - min) + min);
    }
    return ret;
}

MappingFunction::MappingFunction(const std::function<real (real)>& func, real min, real max, uint_fast16_t n_points)
: MappingFunction(evaluateFunction(func, min, max, n_points), min, max)
{
}

MappingFunction::MappingFunction(const std::vector<real>& points, real min, real max)
: points(points)
, min(min)
, max(max)
, step_size((max - min) / (points.size() - 1))
{
}

MappingFunction::real MappingFunction::map(const real in)
{
    if (in <= min)
    {
        return points.front();
    }
    if (in >= max)
    {
        return points.back();
    }
    const real index_value = (in - min) / step_size;
    const size_t index = size_t(index_value);
    assert(index >= 0);
    assert(index + 1 < points.size());
    const real residual = index_value - index;
    assert(residual >= 0.0);
    assert(residual <= 1.0);
    const real before = points[index];
    const real after = points[index + 1];
    return (static_cast<real>(1.0) - residual) * before + residual * after;
}


std::function<MappingFunction::real (MappingFunction::real)> MappingFunction::getFunction()
{ // keeping it real
    return [this](float in) { return this->map(in); };
}
}//namespace cura

