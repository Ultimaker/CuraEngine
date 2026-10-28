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
    AUTOMATIC, // Let retraction be calculated automatically for the travel move
    RETRACTED, // Force travel move to be retracted
    NOT_RETRACTED, // Force travel move not to be retracted
};

} // namespace curaengine
