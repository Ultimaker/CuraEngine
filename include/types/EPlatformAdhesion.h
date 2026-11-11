// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace curaengine::types
{
/*!
 * Type of platform adhesion.
 */
enum class EPlatformAdhesion
{
    Skirt,
    Brim,
    Raft,
    None,
    Plugin,
};

} // namespace curaengine::types
