// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace curaengine::types
{
/*!
 * Whether to force a retracted or unretracted travel move when going to an outer wall
 */
enum class RetractBeforeOuterWall
{
    AUTOMATIC, // Let retraction be calculated automatically for the travel move
    RETRACTED, // Force travel move to be retracted
    NOT_RETRACTED, // Force travel move not to be retracted
    NOT_RETRACTED_FROM_INFILL, // Force the first travel move coming from an infill area not to be retracted, others will be use automatic
};

} // namespace curaengine::types
