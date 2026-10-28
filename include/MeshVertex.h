// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <cstdint>
#include <vector>

#include "geometry/Point3LL.h"

namespace curaengine
{
/*!
Vertex type to be used in a Mesh.

Keeps track of which faces connect to it.
*/
class MeshVertex
{
public:
    ::curaengine::geometry::Point3LL p_; //!< location of the vertex
    std::vector<uint32_t> connected_faces_; //!< list of the indices of connected faces

    MeshVertex(::curaengine::geometry::Point3LL p)
        : p_(p)
    {
        connected_faces_.reserve(8);
    } //!< doesn't set connected_faces
};

} // namespace curaengine
