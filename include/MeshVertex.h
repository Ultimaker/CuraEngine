// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <cstdint>
#include <vector>

#include "geometry/Point3LL.h"

namespace cura
{
/*!
Vertex type to be used in a Mesh.

Keeps track of which faces connect to it.
*/
class MeshVertex
{
public:
    Point3LL p_; //!< location of the vertex
    std::vector<uint32_t> connected_faces_; //!< list of the indices of connected faces

    MeshVertex(Point3LL p)
        : p_(p)
    {
        connected_faces_.reserve(8);
    } //!< doesn't set connected_faces
};

} // namespace cura
