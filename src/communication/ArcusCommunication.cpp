//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ArcusCommunication.h"

namespace cura
{

const bool ArcusCommunication::hasSlice() const
{
    return !to_slice.empty();
}

} //namespace cura