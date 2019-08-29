//Copyright (C) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MESH_GROUP_H
#define MESH_GROUP_H

#include "mesh.h"
#include "utils/NoCopy.h"

namespace cura
{

class FMatrix3x3;

/*!
 * A MeshGroup is a collection with 1 or more 3D meshes.
 * 
 * One MeshGroup is a whole which is printed at once.
 * Generally there is one single MeshGroup, though when using one-at-a-time printing, multiple MeshGroups are processed consecutively.
 */
class MeshGroup : public NoCopy
{
public:
    std::vector<Mesh> meshes;
    Settings settings;

    Point3 min() const; //! minimal corner of bounding box
    Point3 max() const; //! maximal corner of bounding box

    void clear();

    void finalize();
};

/*!
 * Load a Mesh from file and store it in the \p meshgroup.
 * 
 * \param meshgroup The meshgroup where to store the mesh
 * \param filename The filename of the mesh file
 * \param transformation The transformation applied to all vertices
 * \param object_parent_settings (optional) The parent settings object of the new mesh. Defaults to \p meshgroup if none is given.
 * \return whether the file could be loaded
 */
bool loadMeshIntoMeshGroup(MeshGroup* meshgroup, const char* filename, const FMatrix3x3& transformation, Settings& object_parent_settings);

} //namespace cura

#endif //MESH_GROUP_H
