/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef MESH_GROUP_H
#define MESH_GROUP_H

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
class MeshGroup : public SettingsBase
{
    ExtruderTrain* extruders[MAX_EXTRUDERS] = {nullptr};
public:

    MeshGroup(SettingsBaseVirtual* settings_base) : SettingsBase(settings_base){}
    ~MeshGroup() 
    {
        for (unsigned int extruder = 0; extruder < MAX_EXTRUDERS; extruder++)
        {
            if (extruders[extruder])
            {
                delete extruders[extruder];
            }
        }
    }
    
    ExtruderTrain* getExtruderTrain(unsigned int extruder_nr)
    {
        if (!extruders[extruder_nr])
        {
            extruders[extruder_nr] = new ExtruderTrain(this, extruder_nr);
        }
        return extruders[extruder_nr];
    }
    
    std::vector<Mesh> meshes;

    Point3 min() //! minimal corner of bounding box
    {
        if (meshes.size() < 1)
        {
            return Point3(0, 0, 0);
        }
        Point3 ret = meshes[0].min();
        for(unsigned int i=1; i<meshes.size(); i++)
        {
            Point3 v = meshes[i].min();
            ret.x = std::min(ret.x, v.x);
            ret.y = std::min(ret.y, v.y);
            ret.z = std::min(ret.z, v.z);
        }
        return ret;
    }
    Point3 max() //! maximal corner of bounding box
    {
        if (meshes.size() < 1)
        {
            return Point3(0, 0, 0);
        }
        Point3 ret = meshes[0].max();
        for(unsigned int i=1; i<meshes.size(); i++)
        {
            Point3 v = meshes[i].max();
            ret.x = std::max(ret.x, v.x);
            ret.y = std::max(ret.y, v.y);
            ret.z = std::max(ret.z, v.z);
        }
        return ret;
    }

    void clear()
    {
        for(Mesh& m : meshes)
        {
            m.clear();
        }
    }

    void offset(Point3 offset)
    {
        for(Mesh& m : meshes)
        {
            m.offset(offset);
        }
    }

    void finalize()
    {
        // If a mesh position was given, put the mesh at this position in 3D space. 
        Point3 object_offset(getSettingInMicrons("mesh_position_x"), getSettingInMicrons("mesh_position_y"), getSettingInMicrons("mesh_position_z"));
        if (getSettingBoolean("center_object"))
        {
            Point3 object_min = min();
            Point3 object_max = max();
            Point3 object_size = object_max - object_min;
            object_offset += Point3(-object_min.x - object_size.x / 2, -object_min.y - object_size.y / 2, -object_min.z);
        }
        offset(object_offset);
        
        //If the machine settings have been supplied, offset the given position vertices to the center of vertices (0,0,0) is at the bed center.
        if (!getSettingBoolean("machine_center_is_zero"))
        {
            Point3 object_offset = Point3(0, 0, 0);
            object_offset.x = getSettingInMicrons("machine_width") / 2;
            object_offset.y = getSettingInMicrons("machine_depth") / 2;
            offset(object_offset);
        }
    }
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
bool loadMeshIntoMeshGroup(MeshGroup* meshgroup, const char* filename, FMatrix3x3& transformation, SettingsBaseVirtual* object_parent_settings = nullptr);

}//namespace cura
#endif//MESH_GROUP_H
