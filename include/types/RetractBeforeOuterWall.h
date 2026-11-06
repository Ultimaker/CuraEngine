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
    Automatic, // Let retraction be calculated automatically for the travel move
    Retracted, // Force travel move to be retracted
    NotRetracted, // Force travel move not to be retracted
    NotRetractedFromInfill, // Force the first travel move coming from an infill area not to be retracted, others will be use automatic
};

} // namespace curaengine::types
