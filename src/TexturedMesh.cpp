/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */

#include "TexturedMesh.h"

#include <cassert>


namespace cura
{

TexturedMesh::TexturedMesh(SettingsBaseVirtual* sb)
: Mesh(sb)
{
}

void TexturedMesh::addTextureCoord(double x, double y)
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
        face_texture_indices.emplace_back(ti0, ti1, ti2);
        assert(Mesh::faces.size() == face_texture_indices.size());
    }
}

} // namespace cura
