// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

namespace cura
{
enum class TextureArea
{
    Normal = 0, // Area is to be treated as usual
    Preferred = 1, // Area is to be preferred
    Avoid = 2, // Area is to be avoided
};

} // namespace cura
