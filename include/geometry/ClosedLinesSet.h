// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_CLOSED_LINES_SET_H
#define GEOMETRY_CLOSED_LINES_SET_H

#include "geometry/ClosedPolyline.h"
#include "geometry/LinesSet.h"

namespace cura
{

class MixedLinesSet;

/*!
 * \brief Convenience definition for a container that can hold only closed polylines. This makes it
 *        explicit what the lines actually represent.
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

} // namespace cura

#endif // GEOMETRY_CLOSED_LINES_SET_H
