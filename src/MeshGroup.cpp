/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <string.h>
#include <strings.h>
#include <stdio.h>

#include "MeshGroup.h"
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
        else if (*ptr =='\0')
        {
            return ptr;
        }
        ptr++;
        len--;
    }
    return nullptr;
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

void readBMP(Material* mat, const char* filename)
{
    FILE* f = fopen(filename, "rb");
    if (f == nullptr)
    {
        logError("ERROR: couldn't load image file %s.\n", filename);
        return;
    }
    unsigned char info[54];
    fread(info, sizeof(unsigned char), 54, f); // read the 54-byte header

    // extract image height and width from header
    int width = *(int*)&info[18];
    int height = *(int*)&info[22];

    int size = 3 * width * height;
    unsigned char* data = new unsigned char[size]; // allocate 3 bytes per pixel
    fread(data, sizeof(unsigned char), size, f); // read the rest of the data at once
    fclose(f);
    

//     for (int i = 0; i < size; i += 3)
//     {
//         unsigned char tmp = data[i];
//         data[i] = data[i+2];
//         data[i+2] = tmp;
//     } // BGR ==> RGB
    mat->setData(data);
    mat->setWidthHeight(width, height);
}
void loadMatImage(Material* mat, const char* filename)
{
    const char* ext = strrchr(filename, '.');
    if (ext && (strcmp(ext, ".bmp") == 0 || strcmp(ext, ".BMP") == 0))
    {
        readBMP(mat, filename);
    }
    else
    {
        logError("ERROR: trying to load unsupported image. File %s has %s extension.\n", filename, ext);
    }
}

void loadMaterialBase(TexturedMesh* mesh, const char* filename)
{
    FILE* f = fopen(filename, "rt");
    if (f == nullptr)
    {
        logError("ERROR: Couldn't load MTL file %s.\n", filename);
        return;
    }
    char buffer[1024];
    char mat_name [100];
    char mat_file [100];
    char map_type [10];
    Material* last_mat = nullptr;
    while(fgets_(buffer, sizeof(buffer), f))
    {
        if (buffer[0] == '#')
        {
            continue;
        }
        if (sscanf(buffer, "map_%s %s", map_type, mat_file) == 2 // we don't care what type of map it specifies (currently)
            || sscanf(buffer, "bump %s", mat_file) == 1
            || sscanf(buffer, "disp %s", mat_file) == 1
            || sscanf(buffer, "decal %s", mat_file) == 1
            || sscanf(buffer, "refl %s", mat_file) == 1
        )
        {
            std::string parent_dir = std::string(filename).substr(0, std::string(filename).find_last_of("/\\"));
            std::string mtl_file = parent_dir + "/" + mat_file;
            if (last_mat)
            {
                loadMatImage(last_mat, mtl_file.c_str());
            }
        }
        else if (sscanf(buffer, "newmtl %s", mat_name) == 1)
        {
            last_mat = mesh->addMaterial(mat_name);
        }
    }
    fclose(f);
}

bool loadMeshOBJ(TexturedMesh* mesh, const char* filename, const FMatrix3x3& matrix)
{
    FILE* f = fopen(filename, "rt");
    if (f == nullptr)
    {
        return false;
    }
    char buffer[1024];
    FPoint3 vertex;
    Point3 vertex_indices;
    char face_index_buffer_1 [100];
    char face_index_buffer_2 [100];
    char face_index_buffer_3 [100];
    char str_buffer [100];
    while(fgets_(buffer, sizeof(buffer), f))
    {
        if (buffer[0] == '#')
        {
            continue;
        }
        if (sscanf(buffer, "v %f %f %f", &vertex.x, &vertex.y, &vertex.z) == 3)
        {
            Point3 v = matrix.apply(vertex);
            mesh->addVertex(v);
        }
        else if (sscanf(buffer, "f %s %s %s", face_index_buffer_1, face_index_buffer_2, face_index_buffer_3) == 3)
        {
            int normal_vector_index; // unused
            Point3 texture_indices(0, 0, 0); // becomes -1 if no texture data supplied
            int n_scanned_1 = sscanf(face_index_buffer_1, "%d/%d/%d", &vertex_indices.x, &texture_indices.x, &normal_vector_index); 
            int n_scanned_2 = sscanf(face_index_buffer_2, "%d/%d/%d", &vertex_indices.y, &texture_indices.y, &normal_vector_index); 
            int n_scanned_3 = sscanf(face_index_buffer_3, "%d/%d/%d", &vertex_indices.z, &texture_indices.z, &normal_vector_index); 
            if (n_scanned_1 > 0 && n_scanned_2 > 0 && n_scanned_3 > 0)
            {
                mesh->addFace(vertex_indices.x - 1, vertex_indices.y - 1, vertex_indices.z - 1, texture_indices.x - 1, texture_indices.y - 1, texture_indices.z - 1);
                // obj files count vertex indices starting from 1!
            }
        }
        else if (sscanf(buffer, "mtllib %s", str_buffer) == 1)
        {
            std::string parent_dir = std::string(filename).substr(0, std::string(filename).find_last_of("/\\"));
            std::string mtl_file = parent_dir + "/" + str_buffer;
            loadMaterialBase(mesh, mtl_file.c_str());
        }
    }
    fclose(f);
    mesh->finish();
    return true;
}

bool loadMeshIntoMeshGroup(MeshGroup* meshgroup, const char* filename, const FMatrix3x3& transformation, SettingsBaseVirtual* object_parent_settings)
{
    const char* ext = strrchr(filename, '.');
    if (ext && (strcmp(ext, ".stl") == 0 || strcmp(ext, ".STL") == 0))
    {
        Mesh* mesh = new Mesh(object_parent_settings ? object_parent_settings : meshgroup); //If we have object_parent_settings, use them as parent settings. Otherwise, just use meshgroup.
        if (loadMeshSTL(mesh,filename,transformation)) //Load it! If successful...
        {
            meshgroup->meshes.push_back(mesh);
            return true;
        }
        else
        {
            delete mesh;
        }
    }
    else if (ext && (strcmp(ext, ".obj") == 0 || strcmp(ext, ".OBJ") == 0))
    {
        TexturedMesh* mesh = new TexturedMesh(object_parent_settings ? object_parent_settings : meshgroup); //If we have object_parent_settings, use them as parent settings. Otherwise, just use meshgroup.
        if (loadMeshOBJ(mesh,filename,transformation)) //Load it! If successful...
        {
            meshgroup->meshes.push_back(mesh);
            return true;
        }
        else 
        {
            delete mesh;
        }
    }
        
    return false;
}

}//namespace cura
