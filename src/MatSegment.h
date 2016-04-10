/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef MAT_SEGMENT_H
#define MAT_SEGMENT_H

#include "MatCoord.h"

namespace cura
{

/*!
 * Coordinates in a specific texture bitmap 
 */
struct MatSegment
{
    MatCoord start;
    MatCoord end;
    MatSegment() //!< non-initializing constructor
    {}
    MatSegment(MatCoord start, MatCoord end)
    : start(start)
    , end(end)
    {}
};

} // namespace cura

#endif // MAT_SEGMENT_H