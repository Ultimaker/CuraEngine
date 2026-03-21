// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef SLICEDATA_SKINPART_H
#define SLICEDATA_SKINPART_H

#include "geometry/SingleShape.h"

namespace cura
{

/*!
 * A SkinPart is a connected area designated as top and/or bottom skin.
 * Surrounding each non-bridged skin area with an outline may result in better top skins.
 * It's filled during FffProcessor.processSliceData(.) and used in FffProcessor.writeGCode(.) to generate the final gcode.
 */
class SkinPart
{
public:
    SingleShape outline; //!< The skinOutline is the area which needs to be 100% filled to generate a proper top&bottom filling. It's filled by the "skin" module. Includes both
                         //!< roofing and non-roofing.
    Shape skin_fill; //!< The part of the skin which is not roofing.
    Shape roofing_fill; //!< The inner infill which has air directly above
    Shape flooring_fill; //!< The inner infill which has air directly below
};

} // namespace cura

#endif
