/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef TEXTURE_PROCESSING_MAT_COORD_H
#define TEXTURE_PROCESSING_MAT_COORD_H

#include "../utils/FPoint.h"

namespace cura
{

/*!
 * Coordinates in a specific texture bitmap 
 */
struct MatCoord
{
    FPoint coords;
    int mat_id; //!< Material id
    MatCoord() //!< non-initializing constructor
    {}
    MatCoord(FPoint coords, int mat_id) //!< constructor
    : coords(coords)
    , mat_id(mat_id)
    {}
};

} // namespace cura

#endif // TEXTURE_PROCESSING_MAT_COORD_H