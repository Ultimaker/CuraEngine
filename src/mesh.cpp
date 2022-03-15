// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <spdlog/spdlog.h>

#include "mesh.h"
#include "utils/floatpoint.h"

namespace cura
{

static inline uint64_t intersperse3bits(uint64_t x)
{
    x = (x | x << 32) & 0x1f00000000ffff;
    x = (x | x << 16) & 0x1f0000ff0000ff;
    x = (x | x << 8) & 0x100f00f00f00f00f;
    x = (x | x << 4) & 0x10c30c30c30c30c3;
    x = (x | x << 2) & 0x1249249249249249;
    return x;
}

static inline uint64_t hash_point3(const Point3& v)
{
    return intersperse3bits(static_cast<uint64_t>(v.x)) | (intersperse3bits(static_cast<uint64_t>(v.y)) << 1) | (intersperse3bits(static_cast<uint64_t>(v.z)) << 2);
}

HashMap3D::HashMap3D(size_t nvertices)
{
    // 2/3 load factor
    size_t size = (nvertices * 3) / 2;
    bits = 0;
    while (size)
    {
        bits++;
        size >>= 1;
    }
    bits = std::max(bits, min_bits);

    init();
}

void HashMap3D::init()
{
    clear();
    // Number of vertices that can be inserted before reaching the load factor threshold
    free_vertices = (1 << (bits + 1)) / 3;
    mask = (1 << bits) - 1;
    map = std::make_unique<Item[]>(mask + 1); // map slots, value-initialized to 0
}

void HashMap3D::clear()
{
    free_vertices = 0;
    map.reset();
}

HashMap3D::Item HashMap3D::insert(const Point3& v, const std::vector<MeshVertex>& vertices)
{
    // Lookup
    const auto predicate = [&vertices, &v](Item item) { return vertices[item - 1].p == v; };
    const hash_t hash = hash_point3(v);
    const Item& slot = probe(hash, predicate);

    if (slot)
    {
        // Found a vertex equal to v
        return slot - 1;
    }
    else
    {
        constexpr auto const_false = [](Item) { return false; }; // Predicate for probing during insertion
        // Check if a rehash is needed
        if (! free_vertices)
        {
            bits += 2; // growth factor: 4
            init();
            free_vertices -= vertices.size();
            const auto end = static_cast<ptrdiff_t>(vertices.size());
            for (ptrdiff_t i = 0; i < end; i++)
            {
                probe(hash_point3(vertices[i].p), const_false) = i + 1;
            }
        }

        // Insertion
        free_vertices--;
        Item item = vertices.size(); // New index
        probe(hash, const_false) = item + 1;

        return item;
    }
}

template<typename P>
inline HashMap3D::Item& HashMap3D::probe(hash_t hash, const P& predicate)
{
    // Python's dict hash perurbation algorithm https://github.com/python/cpython/blob/main/Objects/dictnotes.txt
    hash_t perturb = hash;
    hash_t idx = hash & mask;
    while (map[idx] && ! predicate(map[idx]))
    {
        perturb >>= 5;
        idx = (5 * idx) + 1 + perturb;
        idx &= mask;
    }
    return map[idx];
}

Mesh::Mesh(size_t face_count) : spatial_map(face_count / 2)
{
    faces.reserve(face_count);
    vertices.reserve(face_count / 2);
}

void Mesh::addFace(Point3& v0, Point3& v1, Point3& v2)
{
    mesh_idx_t vi0 = findIndexOfVertex(v0);
    mesh_idx_t vi1 = findIndexOfVertex(v1);
    mesh_idx_t vi2 = findIndexOfVertex(v2);
    if (vi0 == vi1 || vi1 == vi2 || vi0 == vi2)
        return; // the face has two vertices which get assigned the same location. Don't add the face.

    mesh_idx_t idx = faces.size(); // index of face to be added
    faces.emplace_back(MeshFace{ { vi0, vi1, vi2 }, {} });
    vertices[vi0].connected_faces.push_back(idx);
    vertices[vi1].connected_faces.push_back(idx);
    vertices[vi2].connected_faces.push_back(idx);
}

void Mesh::clear()
{
    spatial_map.clear();
    faces.clear();
    vertices.clear();
}

void Mesh::finish()
{
    // Finish up the mesh, clear the vertex_hash_map, as it's no longer needed from this point on and uses quite a bit of memory.
    spatial_map.clear();
    // Shrink to fit vectors if more than 1/3 is wasted
    if (faces.size() * 3 < 2 * faces.capacity())
    {
        faces.shrink_to_fit();
    }
    if (vertices.size() * 3 < 2 * vertices.capacity())
    {
        vertices.shrink_to_fit();
    }


    // For each face, store which other face is connected with it.
    for (ptrdiff_t i = 0; i < ptrdiff_t(faces.size()); i++)
    {
        MeshFace& face = faces[i];
        // faces are connected via the outside
        face.connected_face_index[0] = getFaceIdxWithPoints(face.vertex_index[0], face.vertex_index[1], i, face.vertex_index[2]);
        face.connected_face_index[1] = getFaceIdxWithPoints(face.vertex_index[1], face.vertex_index[2], i, face.vertex_index[0]);
        face.connected_face_index[2] = getFaceIdxWithPoints(face.vertex_index[2], face.vertex_index[0], i, face.vertex_index[1]);
    }
}

Point3 Mesh::min() const
{
    return aabb.min;
}
Point3 Mesh::max() const
{
    return aabb.max;
}
AABB3D Mesh::getAABB() const
{
    return aabb;
}
void Mesh::expandXY(int64_t offset)
{
    if (offset)
    {
        aabb.expandXY(offset);
    }
}

void Mesh::transform(const FMatrix4x3& transformation)
{
    for (MeshVertex& v : vertices)
    {
        v.p = transformation.apply(v.p);
    }
    aabb.min = transformation.apply(aabb.min);
    aabb.max = transformation.apply(aabb.max);
}


bool Mesh::isPrinted() const
{
    return ! settings.get<bool>("infill_mesh") && ! settings.get<bool>("cutting_mesh") && ! settings.get<bool>("anti_overhang_mesh");
}

inline mesh_idx_t Mesh::findIndexOfVertex(const Point3& v)
{
    const mesh_idx_t new_idx = vertices.size();
    auto idx = spatial_map.insert(v, vertices);
    if (idx == new_idx)
    {
        vertices.emplace_back(v);
        aabb.include(v);
    }
    return idx;
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
inline ptrdiff_t Mesh::getFaceIdxWithPoints(mesh_idx_t idx0, mesh_idx_t idx1, mesh_idx_t notFaceIdx, mesh_idx_t notFaceVertexIdx) const
{
    boost::container::small_vector<mesh_idx_t, 5> candidateFaces; // in case more than two faces meet at an edge, multiple candidates are generated
    for (mesh_idx_t f : vertices[idx0].connected_faces) // search through all faces connected to the first vertex and find those that are also connected to the second
    {
        if (f == notFaceIdx)
        {
            continue;
        }
        if (faces[f].vertex_index[0] == idx1 // && faces[f].vertex_index[1] == idx0 // next face should have the right direction!
            || faces[f].vertex_index[1] == idx1 // && faces[f].vertex_index[2] == idx0
            || faces[f].vertex_index[2] == idx1 // && faces[f].vertex_index[0] == idx0
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

    FPoint3 vn = vertices[idx1].p - vertices[idx0].p;
    FPoint3 n = vn / vn.vSize(); // the normal of the plane in which all normals of faces connected to the edge lie => the normalized normal
    FPoint3 v0 = vertices[idx1].p - vertices[idx0].p;

    // the normals below are abnormally directed! : these normals all point counterclockwise (viewed from idx1 to idx0) from the face, irrespective of the direction of the face.
    FPoint3 n0 = FPoint3(vertices[notFaceVertexIdx].p - vertices[idx0].p).cross(v0);

    if (n0.vSize() <= 0)
    {
        spdlog::debug("Face {} has zero area!", notFaceIdx);
    }

    double smallestAngle = 1000; // more then 2 PI (impossible angle)
    ptrdiff_t bestIdx = -1;

    for (mesh_idx_t candidateFace : candidateFaces)
    {
        ptrdiff_t candidateVertex;
        { // find third vertex belonging to the face (besides idx0 and idx1)
            for (candidateVertex = 0; candidateVertex < 3; candidateVertex++)
                if (faces[candidateFace].vertex_index[candidateVertex] != idx0 && faces[candidateFace].vertex_index[candidateVertex] != idx1)
                    break;
        }

        FPoint3 v1 = vertices[faces[candidateFace].vertex_index[candidateVertex]].p - vertices[idx0].p;
        FPoint3 n1 = v0.cross(v1);

        double dot = n0 * n1;
        double det = n * n0.cross(n1);
        double angle = std::atan2(det, dot);
        if (angle < 0)
            angle += 2 * M_PI; // 0 <= angle < 2* M_PI

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
