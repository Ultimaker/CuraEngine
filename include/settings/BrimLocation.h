// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace cura
{
/*!
 * Brim location, inside, outside or both
 */
enum class BrimLocation
{
    OUTSIDE = 0x01, // Brim only on the outside of the model
    INSIDE = 0x02, // Brim only on the inside of the model
    EVERYWHERE = 0x03, // Brim on both the outside and inside of the model
};

/*!
 * Convenience binary operator to allow testing brim location easily, like (actual_location & BrimLocation::OUTSIDE)
 */
[[maybe_unused]] static int operator&(BrimLocation location1, BrimLocation location2)
{
    return static_cast<int>(location1) & static_cast<int>(location2);
}

} // namespace cura
