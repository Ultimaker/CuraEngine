#include "mesh.h"

const int vertex_meld_distance = MM2INT(0.03);
static inline uint32_t pointHash(Point3& p)
{
    return ((p.x + vertex_meld_distance/2) / vertex_meld_distance) ^ (((p.y + vertex_meld_distance/2) / vertex_meld_distance) << 10) ^ (((p.z + vertex_meld_distance/2) / vertex_meld_distance) << 20);
}

Mesh::Mesh(SettingsBase* parent)
: SettingsBase(parent)
{
}

void Mesh::addFace(Point3& v0, Point3& v1, Point3& v2)
{
    int idx = faces.size();
    faces.emplace_back();
    MeshFace& face = faces[idx];
    face.vertex_index[0] = findIndexOfVertex(v0);
    face.vertex_index[1] = findIndexOfVertex(v1);
    face.vertex_index[2] = findIndexOfVertex(v2);
    vertices[face.vertex_index[0]].connected_faces.push_back(idx);
    vertices[face.vertex_index[1]].connected_faces.push_back(idx);
    vertices[face.vertex_index[2]].connected_faces.push_back(idx);
}

void Mesh::clear()
{
    faces.clear();
    vertices.clear();
    vertex_hash_map.clear();
}

void Mesh::finish()
{
    // Finish up the mesh, clear the vertex_hash_map, as it's no longer needed from this point on and uses quite a bit of memory.
    vertex_hash_map.clear();

    // For each face, store which other face is connected with it.
    for(unsigned int i=0; i<faces.size(); i++)
    {
        MeshFace& face = faces[i];
        face.connected_face_index[0] = getFaceIdxWithPoints(face.vertex_index[0], face.vertex_index[1], i);
        face.connected_face_index[1] = getFaceIdxWithPoints(face.vertex_index[1], face.vertex_index[2], i);
        face.connected_face_index[2] = getFaceIdxWithPoints(face.vertex_index[2], face.vertex_index[0], i);
    }
}

Point3 Mesh::min()
{
    if (vertices.size() < 1)
        return Point3(0, 0, 0);
    Point3 ret = vertices[0].p;
    for(unsigned int i=0; i<vertices.size(); i++)
    {
        ret.x = std::min(ret.x, vertices[i].p.x);
        ret.y = std::min(ret.y, vertices[i].p.y);
        ret.z = std::min(ret.z, vertices[i].p.z);
    }
    return ret;
}
Point3 Mesh::max()
{
    if (vertices.size() < 1)
        return Point3(0, 0, 0);
    Point3 ret = vertices[0].p;
    for(unsigned int i=0; i<vertices.size(); i++)
    {
        ret.x = std::max(ret.x, vertices[i].p.x);
        ret.y = std::max(ret.y, vertices[i].p.y);
        ret.z = std::max(ret.z, vertices[i].p.z);
    }
    return ret;
}

int Mesh::findIndexOfVertex(Point3& v)
{
    uint32_t hash = pointHash(v);
    
    for(unsigned int idx = 0; idx < vertex_hash_map[hash].size(); idx++)
    {
        if ((vertices[vertex_hash_map[hash][idx]].p - v).testLength(vertex_meld_distance))
        {
            return vertex_hash_map[hash][idx];
        }
    }
    vertex_hash_map[hash].push_back(vertices.size());
    vertices.emplace_back(v);
    return vertices.size() - 1;
}

int Mesh::getFaceIdxWithPoints(int idx0, int idx1, int notFaceIdx)
{
    for(int f0 : vertices[idx0].connected_faces)
    {
        if (f0 == notFaceIdx) continue;
        for(int f1 : vertices[idx1].connected_faces)
        {
            if (f1 == notFaceIdx) continue;
            if (f0 == f1) return f0;
        }
    }
    return -1;
}
