// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "mesh.h"

#include <numbers>

#include <spdlog/spdlog.h>

#include "utils/Point3D.h"

namespace cura
{

const int vertex_meld_distance = MM2INT(0.03);
/*!
 * returns a hash for the location, but first divides by the vertex_meld_distance,
 * so that any point within a box of vertex_meld_distance by vertex_meld_distance would get mapped to the same hash.
 */
static inline uint32_t pointHash(const Point3LL& p)
{
    return ((p.x_ + vertex_meld_distance / 2) / vertex_meld_distance) ^ (((p.y_ + vertex_meld_distance / 2) / vertex_meld_distance) << 10)
         ^ (((p.z_ + vertex_meld_distance / 2) / vertex_meld_distance) << 20);
}

Mesh::Mesh(Settings& parent)
    : settings_(parent)
    , has_disconnected_faces(false)
    , has_overlapping_faces(false)
{
}

Mesh::Mesh()
    : settings_()
    , has_disconnected_faces(false)
    , has_overlapping_faces(false)
{
}

void Mesh::addFace(
    const Point3LL& v0,
    const Point3LL& v1,
    const Point3LL& v2,
    const std::optional<Point2F>& uv0,
    const std::optional<Point2F>& uv1,
    const std::optional<Point2F>& uv2)
{
    int vi0 = findIndexOfVertex(v0);
    int vi1 = findIndexOfVertex(v1);
    int vi2 = findIndexOfVertex(v2);
    if (vi0 == vi1 || vi1 == vi2 || vi0 == vi2)
        return; // the face has two vertices which get assigned the same location. Don't add the face.

    int idx = faces_.size(); // index of face to be added
    faces_.emplace_back();
    MeshFace& face = faces_[idx];
    face.vertex_index_[0] = vi0;
    face.vertex_index_[1] = vi1;
    face.vertex_index_[2] = vi2;
    face.uv_coordinates_[0] = uv0;
    face.uv_coordinates_[1] = uv1;
    face.uv_coordinates_[2] = uv2;
    vertices_[face.vertex_index_[0]].connected_faces_.push_back(idx);
    vertices_[face.vertex_index_[1]].connected_faces_.push_back(idx);
    vertices_[face.vertex_index_[2]].connected_faces_.push_back(idx);
}

void Mesh::clear()
{
    faces_.clear();
    vertices_.clear();
    vertex_hash_map_.clear();
}

void Mesh::finish()
{
    // Finish up the mesh, clear the vertex_hash_map, as it's no longer needed from this point on and uses quite a bit of memory.
    vertex_hash_map_.clear();

    // For each face, store which other face is connected with it.
    for (unsigned int i = 0; i < faces_.size(); i++)
    {
        MeshFace& face = faces_[i];
        // faces are connected via the outside
        face.connected_face_index_[0] = getFaceIdxWithPoints(face.vertex_index_[0], face.vertex_index_[1], i, face.vertex_index_[2]);
        face.connected_face_index_[1] = getFaceIdxWithPoints(face.vertex_index_[1], face.vertex_index_[2], i, face.vertex_index_[0]);
        face.connected_face_index_[2] = getFaceIdxWithPoints(face.vertex_index_[2], face.vertex_index_[0], i, face.vertex_index_[1]);
    }
}

Point3LL Mesh::min() const
{
    return aabb_.min_;
}
Point3LL Mesh::max() const
{
    return aabb_.max_;
}
AABB3D Mesh::getAABB() const
{
    return aabb_;
}
void Mesh::expandXY(int64_t offset)
{
    if (offset)
    {
        aabb_.expandXY(offset);
    }
}

void Mesh::transform(const Matrix4x3D& transformation)
{
    for (MeshVertex& v : vertices_)
    {
        v.p_ = transformation.apply(v.p_);
    }
    aabb_.min_ = transformation.apply(aabb_.min_);
    aabb_.max_ = transformation.apply(aabb_.max_);
}


bool Mesh::isPrinted() const
{
    return ! settings_.get<bool>("infill_mesh") && ! settings_.get<bool>("cutting_mesh") && ! settings_.get<bool>("anti_overhang_mesh");
}

bool Mesh::canInterlock() const
{
    return ! settings_.get<bool>("infill_mesh") && ! settings_.get<bool>("anti_overhang_mesh");
}

int Mesh::findIndexOfVertex(const Point3LL& v)
{
    uint32_t hash = pointHash(v);

    for (unsigned int idx = 0; idx < vertex_hash_map_[hash].size(); idx++)
    {
        if ((vertices_[vertex_hash_map_[hash][idx]].p_ - v).testLength(vertex_meld_distance))
        {
            return vertex_hash_map_[hash][idx];
        }
    }
    vertex_hash_map_[hash].push_back(vertices_.size());
    vertices_.emplace_back(v);

    aabb_.include(v);

    return vertices_.size() - 1;
}

/*!
Returns the index of the 'other' face connected to the edge between vertices with indices idx0 and idx1.
In case more than two faces are connected via the same edge, the next face in a counter-clockwise ordering (looking from idx1 to idx0) is returned.

\cond DOXYGEN_EXCLUDE
    [NON-RENDERED COMENTS]
    For two faces abc and abd with normals n and m, we have that:
    \f{eqnarray*}{
    n &=& \frac{ab \times ac}{\|ab \times ac\|}     \\
    m &=& \frac{ab \times ad}{\|ab \times ad\|}     \\
    n \times m &=& \|n\| \cdot \|m\| \mathbf{p} \sin \alpha  \\
    && (\mathbf{p} \perp n \wedge \mathbf{p} \perp m) \\
    \sin \alpha &=& \|n \times m \|
    &=& \left\| \frac{(ab \times ac) \times (ab \times ad)}{\|ab \times ac\| \cdot \|ab \times ad\|}  \right\|    \\
    &=& \left\| \frac{ (ab \cdot (ac \times ad)) ab  }{\|ab \times ac\| \cdot \|ab \times ad\|}  \right\|    \\
    &=&  \frac{ (ab \cdot (ac \times ad)) \left\| ab   \right\| }{\|ab\| \|ac\| \sin bac \cdot \|ab\| \|ad\| \sin bad}    \\
    &=&  \frac{  ab \cdot (ac \times ad)  }{\|ab\| \|ac\| \|ad\|  \sin bac \sin bad}    \\
    \f}}
\endcond

See <a href="http://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors">Direct way of computing clockwise angle between 2 vectors</a>


*/
int Mesh::getFaceIdxWithPoints(int idx0, int idx1, int notFaceIdx, int notFaceVertexIdx) const
{
    std::vector<int> candidateFaces; // in case more than two faces meet at an edge, multiple candidates are generated
    for (int f : vertices_[idx0].connected_faces_) // search through all faces connected to the first vertex and find those that are also connected to the second
    {
        if (f == notFaceIdx)
        {
            continue;
        }
        if (faces_[f].vertex_index_[0] == idx1 // && faces[f].vertex_index[1] == idx0 // next face should have the right direction!
            || faces_[f].vertex_index_[1] == idx1 // && faces[f].vertex_index[2] == idx0
            || faces_[f].vertex_index_[2] == idx1 // && faces[f].vertex_index[0] == idx0
        )
            candidateFaces.push_back(f);
    }

    if (candidateFaces.size() == 0)
    {
        spdlog::debug("Couldn't find face connected to face {}", notFaceIdx);
        if (! has_disconnected_faces)
        {
            spdlog::warn("Mesh has disconnected faces!");
        }
        has_disconnected_faces = true;
        return -1;
    }
    if (candidateFaces.size() == 1)
    {
        return candidateFaces[0];
    }


    if (candidateFaces.size() % 2 == 0)
    {
        spdlog::debug("Edge with uneven number of faces connecting it!({})\n", candidateFaces.size() + 1);
        if (! has_disconnected_faces)
        {
            spdlog::warn("Mesh has disconnected faces!");
        }
        has_disconnected_faces = true;
    }

    Point3D vn = vertices_[idx1].p_ - vertices_[idx0].p_;
    Point3D n = vn / vn.vSize(); // the normal of the plane in which all normals of faces connected to the edge lie => the normalized normal
    Point3D v0 = vertices_[idx1].p_ - vertices_[idx0].p_;

    // the normals below are abnormally directed! : these normals all point counterclockwise (viewed from idx1 to idx0) from the face, irrespective of the direction of the face.
    Point3D n0 = Point3D(vertices_[notFaceVertexIdx].p_ - vertices_[idx0].p_).cross(v0);

    if (n0.vSize() <= 0)
    {
        spdlog::debug("Face {} has zero area!", notFaceIdx);
    }

    double smallestAngle = 1000; // more then 2 PI (impossible angle)
    int bestIdx = -1;

    for (int candidateFace : candidateFaces)
    {
        int candidateVertex;
        { // find third vertex belonging to the face (besides idx0 and idx1)
            for (candidateVertex = 0; candidateVertex < 3; candidateVertex++)
                if (faces_[candidateFace].vertex_index_[candidateVertex] != idx0 && faces_[candidateFace].vertex_index_[candidateVertex] != idx1)
                    break;
        }

        Point3D v1 = vertices_[faces_[candidateFace].vertex_index_[candidateVertex]].p_ - vertices_[idx0].p_;
        Point3D n1 = v0.cross(v1);

        double dot = n0 * n1;
        double det = n * n0.cross(n1);
        double angle = std::atan2(det, dot);
        if (angle < 0)
            angle += 2 * std::numbers::pi; // 0 <= angle < 2* std::numbers::pi

        if (angle == 0)
        {
            spdlog::debug("Overlapping faces: face {} and face {}.", notFaceIdx, candidateFace);
            if (! has_overlapping_faces)
            {
                spdlog::warn("Mesh has overlapping faces!");
            }
            has_overlapping_faces = true;
        }
        if (angle < smallestAngle)
        {
            smallestAngle = angle;
            bestIdx = candidateFace;
        }
    }
    if (bestIdx < 0)
    {
        spdlog::debug("Couldn't find face connected to face {}.", notFaceIdx);
        if (! has_disconnected_faces)
        {
            spdlog::warn("Mesh has disconnected faces!");
        }
        has_disconnected_faces = true;
    }
    return bestIdx;
}

} // namespace cura
