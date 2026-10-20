// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace cura
{
/*!
 * Prime tower generation mode
 */
enum class PrimeTowerMode
{
    /*!
     * Prime tower that minimizes time and used filament as much as possible.
     */
    INTERLEAVED,

    /*!
     * Prime tower that minimizes time and used filament, but doesn't allow
     * for printing two different filaments over each other.
     */
    NORMAL,
};

} // namespace cura
