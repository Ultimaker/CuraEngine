/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef TEXTURE_PROCESSING_COLOR_H
#define TEXTURE_PROCESSING_COLOR_H

#include <cmath> // pow

namespace cura
{

/*!
 * color functions
 */
class Color
{
public:
    static float degamma(float gamma_corrected_input, float gamma = 2.2)
    {
        return std::pow(gamma_corrected_input, gamma);
    }
};

} // namespace cura

#endif // TEXTURE_PROCESSING_COLOR_H
