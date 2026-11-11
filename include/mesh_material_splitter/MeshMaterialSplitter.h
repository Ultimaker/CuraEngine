// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

namespace curaengine
{

class Mesh;
class MeshGroup;

namespace mesh_material_splitter
{

/*!
 * Generate modifier mesh based user-painted texture data, either for multi-material or support customization
 * @param meshgroup The group to take the sliced meshes from and add the modifier meshes to
 */
void makePaintingModifierMeshes(MeshGroup* meshgroup);

} // namespace mesh_material_splitter

} // namespace curaengine
