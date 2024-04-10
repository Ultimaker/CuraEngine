// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_OPEN_LINES_SET_H
#define GEOMETRY_OPEN_LINES_SET_H

#include "geometry/lines_set.h"
#include "geometry/open_polyline.h"

namespace cura
{

/*!
 * \brief Convenience definition for a container that can hold only open polylines. This makes it
 *        explicit what the lines actually represent.
 */
using OpenLinesSet = LinesSet<OpenPolyline>;
#warning Try with a using instead
/*class OpenLinesSet : public LinesSet<OpenPolyline>
{
public:
    OpenLinesSet() = default;

    OpenLinesSet(const LinesSet<OpenPolyline>& other)
        : LinesSet(other)
    {
    }
};*/

} // namespace cura

#endif // GEOMETRY_OPEN_LINES_SET_H
