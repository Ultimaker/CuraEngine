// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace cura
{
/*!
 * How the draft shield height is limited.
 */
enum class DraftShieldHeightLimitation
{
    FULL, // Draft shield takes full height of the print.
    LIMITED, // Draft shield is limited by draft_shield_height setting.
    PLUGIN,
};

} // namespace cura
