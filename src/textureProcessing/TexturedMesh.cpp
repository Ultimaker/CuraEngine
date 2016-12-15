/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */

#include "TexturedMesh.h"

#include <cassert>
#include "../utils/logoutput.h"

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


bool TexturedMesh::getFaceEdgeMatCoord(unsigned int face_idx, int64_t z, unsigned int p0_idx, unsigned int p1_idx, MatCoord& result) const
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
    const MeshFace& face = faces[face_idx];
    Point3 p0(vertices[face.vertex_index[p0_idx]].p);
    Point3 p1(vertices[face.vertex_index[p1_idx]].p);

    float dzp0 = z - p0.z;
    float dp0p1 = p1.z - p0.z;

    if (dzp0 * dp0p1 < 0)
    { // z doesn't lie between p0 and p1
        return false;
    }
    if (dzp0 == 0)
    { // edge is not cut by horizontal plane!
        return false;
    }
    float ratio = INT2MM(dzp0) / INT2MM(dp0p1);

    FPoint t0 = texture_coords[texture_idxs.index[p0_idx]];
    FPoint t1 = texture_coords[texture_idxs.index[p1_idx]];

    result.mat = material_base.getMat(texture_idxs.mat_id);
    result.coords.x = t0.x + (t1.x - t0.x) * ratio;
    result.coords.y = t0.y + (t1.y - t0.y) * ratio;

    if (result.coords.x > 1.001 || result.coords.x < -0.001 || result.coords.y > 1.001 || result.coords.y < -0.001)
    {
        logError("WARNING: wapping material to outside image!");
    }
    return true;
}

bool TexturedMesh::sliceFaceTexture(unsigned int face_idx, unsigned int idx_shared, unsigned int idx_first, unsigned int idx_second, int32_t z, Point segment_start, Point segment_end, MatSegment& result) const
{
    if (!getFaceEdgeMatCoord(face_idx, z, idx_shared, idx_first, result.start))
    {
        return false;
    }
    if (!getFaceEdgeMatCoord(face_idx, z, idx_shared, idx_second, result.end))
    {
        return false;
    }
    return true;
}


} // namespace cura
