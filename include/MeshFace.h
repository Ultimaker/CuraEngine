// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <optional>

#include "geometry/Point2F.h"

namespace curaengine
{
/*! A MeshFace is a 3 dimensional model triangle with 3 points. These points are already converted to integers

A face has 3 connected faces, corresponding to its 3 edges.

Note that a correct model may have more than 2 faces connected via a single edge!
In such a case the face_index stored in connected_face_index is the one connected via the outside; see ASCII art below:

: horizontal slice through vertical edge connected to four faces :

\verbatim
[inside] x|
         x| <--+--- faces which contain each other in their connected_face_index fiels
   xxxxxxx|   \|/
   -------+-------
      ^   |xxxxxxx
      +-->|x
      |   |x [inside]
      |
    faces which contain each other in their connected_face_index fiels
\endverbatim
*/
class MeshFace
{
public:
    int vertex_index_[3] = { -1 }; //!< counter-clockwise ordering
    int connected_face_index_[3]; //!< same ordering as vertex_index (connected_face 0 is connected via vertex 0 and 1, etc.)
    std::optional<::curaengine::geometry::Point2F> uv_coordinates_[3]; //!< UV coordinates for each vertex of the face
};

} // namespace curaengine
