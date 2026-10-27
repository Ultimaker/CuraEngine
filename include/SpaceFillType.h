// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once


namespace cura
{

/*!
 * Enum class enumerating the strategies with which an area can be occupied with filament
 *
 * The walls/perimeters are Polygons
 * ZigZag infill is PolyLines, and so is following mesh surface mode for non-polygon surfaces
 * Grid, Triangles and lines infill is Lines
 */
enum class SpaceFillType
{
    None,
    Polygons,
    PolyLines,
    Lines
};

} // namespace cura
