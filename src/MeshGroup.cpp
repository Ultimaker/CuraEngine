//Copyright (C) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <limits>

#include "MeshGroup.h"
#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "utils/string.h"

namespace cura
{

FILE* binaryMeshBlob = nullptr;

/* Custom fgets function to support Mac line-ends in Ascii STL files. OpenSCAD produces this when used on Mac */
void* fgets_(char* ptr, size_t len, FILE* f)
{
    while(len && fread(ptr, 1, 1, f) > 0)
    {
        if (*ptr == '\n' || *ptr == '\r')
        {
            *ptr = '\0';
            return ptr;
        }
        ptr++;
        len--;
    }
    return nullptr;
}

Point3 MeshGroup::min() const
{
    if (meshes.size() < 1)
    {
        return Point3(0, 0, 0);
    }
    Point3 ret(std::numeric_limits<coord_t>::max(), std::numeric_limits<coord_t>::max(), std::numeric_limits<coord_t>::max());
    for (const Mesh& mesh : meshes)
    {
        if (mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("cutting_mesh") || mesh.settings.get<bool>("anti_overhang_mesh")) //Don't count pieces that are not printed.
        {
            continue;
        }
        Point3 v = mesh.min();
        ret.x = std::min(ret.x, v.x);
        ret.y = std::min(ret.y, v.y);
        ret.z = std::min(ret.z, v.z);
    }
    return ret;
}

Point3 MeshGroup::max() const
{
    if (meshes.size() < 1)
    {
        return Point3(0, 0, 0);
    }
    Point3 ret(std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min());
    for (const Mesh& mesh : meshes)
    {
        if (mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("cutting_mesh") || mesh.settings.get<bool>("anti_overhang_mesh")) //Don't count pieces that are not printed.
        {
            continue;
        }
        Point3 v = mesh.max();
        ret.x = std::max(ret.x, v.x);
        ret.y = std::max(ret.y, v.y);
        ret.z = std::max(ret.z, v.z);
    }
    return ret;
}

void MeshGroup::clear()
{
    for(Mesh& m : meshes)
    {
        m.clear();
    }
}

void MeshGroup::finalize()
{
    //If the machine settings have been supplied, offset the given position vertices to the center of vertices (0,0,0) is at the bed center.
    Point3 meshgroup_offset(0, 0, 0);
    if (!settings.get<bool>("machine_center_is_zero"))
    {
        meshgroup_offset.x = settings.get<coord_t>("machine_width") / 2;
        meshgroup_offset.y = settings.get<coord_t>("machine_depth") / 2;
    }
    
    // If a mesh position was given, put the mesh at this position in 3D space. 
    for(Mesh& mesh : meshes)
    {
        Point3 mesh_offset(mesh.settings.get<coord_t>("mesh_position_x"), mesh.settings.get<coord_t>("mesh_position_y"), mesh.settings.get<coord_t>("mesh_position_z"));
        if (mesh.settings.get<bool>("center_object"))
        {
            Point3 object_min = mesh.min();
            Point3 object_max = mesh.max();
            Point3 object_size = object_max - object_min;
            mesh_offset += Point3(-object_min.x - object_size.x / 2, -object_min.y - object_size.y / 2, -object_min.z);
        }
        mesh.offset(mesh_offset + meshgroup_offset);
    }
}

bool loadMeshSTL_ascii(Mesh* mesh, const char* filename, const FMatrix3x3& matrix)
{
    FILE* f = fopen(filename, "rt");
    char buffer[1024];
    FPoint3 vertex;
    int n = 0;
    Point3 v0(0,0,0), v1(0,0,0), v2(0,0,0);
    while(fgets_(buffer, sizeof(buffer), f))
    {
        if (sscanf(buffer, " vertex %f %f %f", &vertex.x, &vertex.y, &vertex.z) == 3)
        {
            n++;
            switch(n)
            {
            case 1:
                v0 = matrix.apply(vertex);
                break;
            case 2:
                v1 = matrix.apply(vertex);
                break;
            case 3:
                v2 = matrix.apply(vertex);
                mesh->addFace(v0, v1, v2);
                n = 0;
                break;
            }
        }
    }
    fclose(f);
    mesh->finish();
    return true;
}

bool loadMeshSTL_binary(Mesh* mesh, const char* filename, const FMatrix3x3& matrix)
{
    FILE* f = fopen(filename, "rb");

    fseek(f, 0L, SEEK_END);
    long long file_size = ftell(f); //The file size is the position of the cursor after seeking to the end.
    rewind(f); //Seek back to start.
    size_t face_count = (file_size - 80 - sizeof(uint32_t)) / 50; //Subtract the size of the header. Every face uses exactly 50 bytes.

    char buffer[80];
    //Skip the header
    if (fread(buffer, 80, 1, f) != 1)
    {
        fclose(f);
        return false;
    }

    uint32_t reported_face_count;
    //Read the face count. We'll use it as a sort of redundancy code to check for file corruption.
    if (fread(&reported_face_count, sizeof(uint32_t), 1, f) != 1)
    {
        fclose(f);
        return false;
    }
    if (reported_face_count != face_count)
    {
        logWarning("Face count reported by file (%s) is not equal to actual face count (%s). File could be corrupt!\n", std::to_string(reported_face_count).c_str(), std::to_string(face_count).c_str());
    }

    //For each face read:
    //float(x,y,z) = normal, float(X,Y,Z)*3 = vertexes, uint16_t = flags
    // Every Face is 50 Bytes: Normal(3*float), Vertices(9*float), 2 Bytes Spacer
    mesh->faces.reserve(face_count);
    mesh->vertices.reserve(face_count);
    for (unsigned int i = 0; i < face_count; i++)
    {
        if (fread(buffer, 50, 1, f) != 1)
        {
            fclose(f);
            return false;
        }
        float *v= ((float*)buffer)+3;

        Point3 v0 = matrix.apply(FPoint3(v[0], v[1], v[2]));
        Point3 v1 = matrix.apply(FPoint3(v[3], v[4], v[5]));
        Point3 v2 = matrix.apply(FPoint3(v[6], v[7], v[8]));
        mesh->addFace(v0, v1, v2);
    }
    fclose(f);
    mesh->finish();
    return true;
}

bool loadMeshSTL(Mesh* mesh, const char* filename, const FMatrix3x3& matrix)
{
    FILE* f = fopen(filename, "r");
    if (f == nullptr)
    {
        return false;
    }
    
    //assign filename to mesh_name
    mesh->mesh_name = filename;
    
    //Skip any whitespace at the beginning of the file.
    unsigned long long num_whitespace = 0; //Number of whitespace characters.
    unsigned char whitespace;
    if (fread(&whitespace, 1, 1, f) != 1)
    {
        fclose(f);
        return false;
    }
    while(isspace(whitespace))
    {
        num_whitespace++;
        if (fread(&whitespace, 1, 1, f) != 1)
        {
            fclose(f);
            return false;
        }
    }
    fseek(f, num_whitespace, SEEK_SET); //Seek to the place after all whitespace (we may have just read too far).

    char buffer[6];
    if (fread(buffer, 5, 1, f) != 1)
    {
        fclose(f);
        return false;
    }
    fclose(f);

    buffer[5] = '\0';
    if (stringcasecompare(buffer, "solid") == 0)
    {
        bool load_success = loadMeshSTL_ascii(mesh, filename, matrix);
        if (!load_success)
            return false;

        // This logic is used to handle the case where the file starts with
        // "solid" but is a binary file.
        if (mesh->faces.size() < 1)
        {
            mesh->clear();
            return loadMeshSTL_binary(mesh, filename, matrix);
        }
        return true;
    }
    return loadMeshSTL_binary(mesh, filename, matrix);
}

bool loadMeshIntoMeshGroup(MeshGroup* meshgroup, const char* filename, const FMatrix3x3& transformation, Settings& object_parent_settings)
{
    TimeKeeper load_timer;

    const char* ext = strrchr(filename, '.');
    if (ext && (strcmp(ext, ".stl") == 0 || strcmp(ext, ".STL") == 0))
    {
        Mesh mesh(object_parent_settings);
        if (loadMeshSTL(&mesh, filename, transformation)) //Load it! If successful...
        {
            meshgroup->meshes.push_back(mesh);
            log("loading '%s' took %.3f seconds\n", filename, load_timer.restart());
            return true;
        }
    }
    return false;
}

}//namespace cura
