// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace curaengine::types
{
/*!
 * Type of support material to generate
 */
enum class ESupportType
{
    None,
    PlatformOnly,
    Everywhere,
    Plugin,
};

} // namespace curaengine::types
