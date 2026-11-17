// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

namespace curaengine
{

/*!
 * Whether to force a retracted or unretracted travel move
 */
enum class ForceRetract
{
    Automatic, // Let retraction be calculated automatically for the travel move
    Retracted, // Force travel move to be retracted
    NotRetracted, // Force travel move not to be retracted
};

} // namespace curaengine
