/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef TEXTURE_PROCESSING_MAT_COORD_H
#define TEXTURE_PROCESSING_MAT_COORD_H

#include "../utils/FPoint.h"

#include "Material.h"

namespace cura
{

/*!
 * Coordinates in a specific texture bitmap 
 */
struct MatCoord
{
    FPoint coords;
    const Material* mat; //!< Material id
    MatCoord() //!< non-initializing constructor
    {}
    MatCoord(FPoint coords, const Material& mat) //!< constructor
    : coords(coords)
    , mat(&mat)
    {}

    /*!
     * Get the color of the material to which this coordinate is pointing
     */
    float getColor(ColourUsage color) const
    {
        if (mat)
        {
            return mat->getColor(coords.x, coords.y, color);
        }
        else
        {
            return 0.0f;
        }
    }
};

} // namespace cura

#endif // TEXTURE_PROCESSING_MAT_COORD_H