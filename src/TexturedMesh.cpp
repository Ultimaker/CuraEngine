/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */

#include "TexturedMesh.h"

#include <cassert>


namespace cura
{

TexturedMesh::TexturedMesh(SettingsBaseVirtual* sb)
: Mesh(sb)
, current_mat(-1) // not set yet
{
}

void TexturedMesh::addTextureCoord(float x, float y)
{
    texture_coords.emplace_back(x, y);
}

void TexturedMesh::addFace(int vi0, int vi1, int vi2, int ti0, int ti1, int ti2)
{
    if (vi0 < -1)
    {
        vi0 = Mesh::faces.size() + vi0 + 1; // + 1 because of relative indexing doesn't start counting from 1
    }
    if (vi1 < -1)
    {
        vi1 = Mesh::faces.size() + vi0 + 1; // + 1 because of relative indexing doesn't start counting from 1
    }
    if (vi2 < -1)
    {
        vi2 = Mesh::faces.size() + vi0 + 1; // + 1 because of relative indexing doesn't start counting from 1
    }
    if (ti0 < -1)
    {
        ti0 = texture_coords.size() + vi0 + 1; // + 1 because of relative indexing doesn't start counting from 1
    }
    if (ti1 < -1)
    {
        ti1 = texture_coords.size() + vi0 + 1; // + 1 because of relative indexing doesn't start counting from 1
    }
    if (ti2 < -1)
    {
        ti2 = texture_coords.size() + vi0 + 1; // + 1 because of relative indexing doesn't start counting from 1
    }
    bool made_new_face = Mesh::addFace(vi0, vi1, vi2);
    if (made_new_face)
    {
        face_texture_indices.emplace_back(ti0, ti1, ti2, current_mat);
        assert(Mesh::faces.size() == face_texture_indices.size());
    }
}

bool TexturedMesh::setMaterial(std::string name)
{
    current_mat = material_base.getMatId(name);
    return current_mat >= 0;
}

Material* TexturedMesh::addMaterial(std::__cxx11::string name)
{
    return material_base.add(name);
}

/*
bool TexturedMesh::getMatCoord(unsigned int face_idx, const Point3 loc, TexturedMesh::MatCoord& result)
{
    if (face_idx >= face_texture_indices.size() || face_idx >= faces.size())
    {
        return false;
    }
    FaceTextureCoordIndices texture_idxs = face_texture_indices[face_idx];
    if (texture_idxs.i1 < 0 || texture_idxs.i2 < 0 || texture_idxs.i2 < 0 || texture_idxs.mat_id < 0)
    {
        return false;
    }
    FPoint3 x(loc);

    MeshFace& face = faces[face_idx];
    FPoint3 a(vertices[face.vertex_index[0]].p);
    FPoint3 b(vertices[face.vertex_index[1]].p);
    FPoint3 c(vertices[face.vertex_index[2]].p);
    
    FPoint3 ab = b - a;
    FPoint3 ax = x - a;

    Coord t1 = texture_coords[texture_idxs.i1];
    Coord t2 = texture_coords[texture_idxs.i2];
    Coord t3 = texture_coords[texture_idxs.i3];
    FPoint3 ap(t1.x, t1.y, 0.0f);
    FPoint3 bp(t2.x, t2.y, 0.0f);
    FPoint3 cp(t3.x, t3.y, 0.0f);

    FPoint3 apbp = bp - ap;

//     float cosa = ax * ab; // dot product
//     FPoint3 x_p_ab = ; // x projected onto ab
}
*/

bool TexturedMesh::getFaceEdgeMatCoord(unsigned int face_idx, int64_t z, unsigned int p0_idx, unsigned int p1_idx, Coord& result)
{
    if (face_idx >= face_texture_indices.size() || face_idx >= faces.size())
    {
        return false;
    }
    FaceTextureCoordIndices texture_idxs = face_texture_indices[face_idx];
    if (texture_idxs.index[0] < 0 || texture_idxs.index[1] < 0 || texture_idxs.index[2] < 0 || texture_idxs.mat_id < 0)
    {
        return false;
    }
    MeshFace& face = faces[face_idx];
    FPoint3 p0(vertices[face.vertex_index[p0_idx]].p);
    FPoint3 p1(vertices[face.vertex_index[p1_idx]].p);

    float dzp0 = z - p0.z;
    float dp0p1 = p1.z - p0.z;

    if (dzp0 * dp0p1 < 0.0f)
    { // z doesn't lie between p0 and p1
        return false;
    }
    if (dzp0 == 0)
    { // edge is not cut by horizontal plane!
        return false;
    }
    float ratio = dp0p1 / dzp0;

    Coord t0 = texture_coords[texture_idxs.index[p0_idx]];
    Coord t1 = texture_coords[texture_idxs.index[p1_idx]];

    result.x = t0.x + (t1.x - t0.x) * ratio;
    result.y = t0.y + (t1.y - t0.y) * ratio;

    return true;
}

} // namespace cura
