// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_POLYLINE_TYPE_H
#define UTILS_POLYLINE_TYPE_H

namespace cura
{

enum class PolylineType
{
    Open, // Line is open and has no wise
    ImplicitelyClosed, // Line is closed by having a virtual additional segment between last and first vertices
    ExplicitelyClosed, // Line is closed by having the same point twice at beginning and end of list
    Filled
};

} // namespace cura

#endif // UTILS_POLYLINE_TYPE_H
