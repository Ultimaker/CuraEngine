// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_CLOSED_LINES_SET_H
#define GEOMETRY_CLOSED_LINES_SET_H

#include "geometry/closed_polyline.h"
#include "geometry/lines_set.h"

namespace cura
{

/*!
 * \brief Convenience definition for a container that can hold only closed polylines. This makes it
 *        explicit what the lines actually represent.
 */
#warning Try with a using instead
class ClosedLinesSet : public LinesSet<ClosedPolyline>
{
public:
    ClosedLinesSet() = default;
};

} // namespace cura

#endif // GEOMETRY_CLOSED_LINES_SET_H
