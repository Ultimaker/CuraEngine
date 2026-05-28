// Copyright (c) 2026 UltiMaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef FORCERETRACT_H
#define FORCERETRACT_H

namespace cura
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

} // namespace cura

#endif
