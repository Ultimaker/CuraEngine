// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TEXTUREDATAMAPPING_H
#define TEXTUREDATAMAPPING_H

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

enum class TextureArea
{
    Normal = 0, // Area is to be treated as usual
    Preferred = 1, // Area is to be preferred
    Avoid = 2, // Area is to be avoided
};

} // namespace cura
#endif // MESH_H
