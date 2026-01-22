// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MESH_MATERIAL_SPLITTER_H
#define MESH_MATERIAL_SPLITTER_H

namespace cura
{

class Mesh;
class MeshGroup;

namespace MeshMaterialSplitter
{

/*!
 * Generate a modifier mesh for every extruder other than 0, that has some user-painted texture data
 * @param meshgroup The group to take the sliced meshes from and add the modifier meshes to
 */
void makeMaterialModifierMeshes(MeshGroup* meshgroup);

} // namespace MeshMaterialSplitter

} // namespace cura

#endif // MESH_MATERIAL_SPLITTER_H
