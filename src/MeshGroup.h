/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef MESH_GROUP_H
#define MESH_GROUP_H

#include "utils/NoCopy.h"
#include "mesh.h"
#include "ExtruderTrain.h"

namespace cura
{
    
/*!
 * A MeshGroup is a collection with 1 or more 3D meshes.
 * 
 * One MeshGroup is a whole which is printed at once.
 * Generally there is one single MeshGroup, though when using one-at-a-time printing, multiple MeshGroups are processed consecutively.
 */
class MeshGroup : public SettingsBase, NoCopy
{
    ExtruderTrain* extruders[MAX_EXTRUDERS] = {nullptr};
    mutable int extruder_count; //!< The number of extruders. (mutable because of lazy evaluation)
public:
    int getExtruderCount() const;

    MeshGroup(SettingsBaseVirtual* settings_base);
    
    ~MeshGroup();
    
    /*!
     * Create a new extruder train for the @p extruder_nr, or return the one which already exists.
     */
    ExtruderTrain* createExtruderTrain(unsigned int extruder_nr);

    ExtruderTrain* getExtruderTrain(unsigned int extruder_nr);

    const ExtruderTrain* getExtruderTrain(unsigned int extruder_nr) const;

    std::vector<Mesh> meshes;

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
bool loadMeshIntoMeshGroup(MeshGroup* meshgroup, const char* filename, const FMatrix3x3& transformation, SettingsBaseVirtual* object_parent_settings = nullptr);

}//namespace cura
#endif//MESH_GROUP_H
