// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "geometry/ClosedPolyline.h"
#include "geometry/LinesSet.h"

namespace curaengine::geometry
{

class MixedLinesSet;

/*!
 * \brief Container that can hold only closed polylines. This makes it explicit what the lines actually represent and adds some processing functions that can only be applied
 *        to closes polylines.
 */
class ClosedLinesSet : public LinesSet<ClosedPolyline>
{
public:
    ClosedLinesSet() = default;

    ClosedLinesSet(LinesSet<ClosedPolyline>&& other)
        : LinesSet(std::move(other))
    {
    }

    explicit ClosedLinesSet(const std::initializer_list<ClosedPolyline>& initializer)
        : LinesSet(initializer)
    {
    }

    explicit ClosedLinesSet(const ClosedPolyline& line)
        : LinesSet(line)
    {
    }

    [[nodiscard]] MixedLinesSet intersection(const Shape& shape) const;

    [[nodiscard]] MixedLinesSet difference(const Shape& shape) const;
};

} // namespace curaengine::geometry
