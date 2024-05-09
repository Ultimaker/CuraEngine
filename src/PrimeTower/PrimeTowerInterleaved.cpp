// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PrimeTower/PrimeTowerInterleaved.h"

namespace cura
{

PrimeTowerInterleaved::PrimeTowerInterleaved(SliceDataStorage& storage, size_t extruder_count)
    : PrimeTower(storage, extruder_count)
{
}

} // namespace cura
