/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */

#include "TexturedMesh.h"


namespace cura
{

void TexturedMesh::addTextureCoord(double x, double y)
{
    texture_coords.emplace_back(x, y);
}

void TexturedMesh::addFace(int vi0, int vi1, int vi2, int ti1, int ti2, int ti3)
{
    Mesh::addFace(vi0, vi1, vi2);
    //face_texture_indices
}

} // namespace cura
