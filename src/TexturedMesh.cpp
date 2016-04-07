/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */

#include "TexturedMesh.h"

#include <cassert>


namespace cura
{

void TexturedMesh::addTextureCoord(double x, double y)
{
    texture_coords.emplace_back(x, y);
}

void TexturedMesh::addFace(int vi0, int vi1, int vi2, int ti1, int ti2, int ti3)
{
    bool made_new_face = Mesh::addFace(vi0, vi1, vi2);
    if (made_new_face)
    {
        face_texture_indices.emplace_back(ti1, ti2, ti3);
        assert(Mesh::faces.size() == face_texture_indices.size());
    }
}

} // namespace cura
