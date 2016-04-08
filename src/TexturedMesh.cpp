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
    MeshFace& face = faces[face_idx];

    Coord t1 = texture_coords[texture_idxs.i1];
    FPoint3 t1p(t1.x, t1.y, 0.0f);
    FPoint3 p1 = vertices[face.vertex_index[0]].p;
}


} // namespace cura
