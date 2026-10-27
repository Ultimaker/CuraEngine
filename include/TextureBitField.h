// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <cstddef>
#include <map>
#include <string>

namespace cura
{

/*!
 * Describes a bit field in a pixel of a texture, as many different features may be included inside a single pixel
 * For more details, see https://github.com/Ultimaker/Cura/wiki/Painting-data-storage
 */
struct TextureBitField
{
    size_t bit_range_start_index{ 0 }; // The index of the first bit of the field
    size_t bit_range_end_index{ 0 }; // The index of the last bit of the field
};

/*!
 * Gives the bit fields description of every feature stored in the texture
 */
using TextureDataMapping = std::map<std::string, TextureBitField>;

} // namespace cura
