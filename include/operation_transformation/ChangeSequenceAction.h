// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_CHANGESEQUENCEACTION_H
#define PATHPROCESSING_CHANGESEQUENCEACTION_H

namespace cura
{

enum class ChangeSequenceAction
{
    None, // Nothing to do, point is already the start point
    Reverse, // Reverse the (open) extrusion sequence, point is the last one
    Reorder // Reorder the (closed) extrusion sequence so that it starts/ends with the point
};

} // namespace cura

#endif // PATHPROCESSING_CHANGESEQUENCEACTION_H
