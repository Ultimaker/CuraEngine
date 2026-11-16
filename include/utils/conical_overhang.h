// Copyright (c) 2026 Tim Kuipers
// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

namespace curaengine
{

class Mesh;
class Slicer;

/*!
 * Functions for changing the geometry of a model such that it is printable without support -
 * Or at least with at least support as possible
 */
namespace utils::conical_overhang
{
/*!
 * Change the slice data such that the model becomes more printable
 *
 * \param[in,out] slicer The slice data.
 * \param mesh The mesh to get the settings from.
 */
void apply(Slicer* slicer, const Mesh& mesh);
} // namespace utils::conical_overhang

} // namespace curaengine
